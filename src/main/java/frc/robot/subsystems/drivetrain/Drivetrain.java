package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.ReefStalk;


public class Drivetrain extends SubsystemBase {

    private GyroIO gyroIO;
    private GyroIOInputsAutoLogged gyroInputs;

    private GyroIOPigeon pigeon = new GyroIOPigeon();


    private boolean fullyTrustVisionNextPoseUpdate = false;
    private boolean allowTeleportsNextPoseUpdate = false;
    private boolean hasAcceptablePoseObservationsThisLoop = false;

    private SwerveModule[] swerveModules;

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    private SwerveDrivePoseEstimator wheelsOnlyPoseEstimator;

    /** error measured in degrees, output is in degrees per second. */
    private PIDController angleController;

    /** error measured in meters, output is in meters per second. */
    private PIDController translationController;


    // in meters/sec
    private ProfiledPIDController profiledController;

    private TrapezoidProfile motionProfile;

    /** used to rotate about the intake instead of the center of the robot */
    private Transform2d centerOfRotation_robotFrame = new Transform2d();
    
    public Drivetrain(
        GyroIO gyroIO, 
        SwerveModuleIO flSwerveModuleIO, 
        SwerveModuleIO frSwerveModuleIO, 
        SwerveModuleIO blSwerveModuleIO, 
        SwerveModuleIO brSwerveModuleIO
    ) {
        this.gyroIO = gyroIO;
        gyroInputs = new GyroIOInputsAutoLogged();

        swerveModules = new SwerveModule[] {
            new SwerveModule(flSwerveModuleIO, 0, "frontLeft"),
            new SwerveModule(frSwerveModuleIO, 1, "frontRight"),
            new SwerveModule(blSwerveModuleIO, 2, "backLeft"),
            new SwerveModule(brSwerveModuleIO, 3, "backRight")
        };

        gyroIO.setRobotYaw(0);

        //corresponds to x, y, and rotation standard deviations (meters and radians)
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.005);

        
        //corresponds to x, y, and rotation standard deviations (meters and radians)
        //these values are automatically recalculated periodically depending on distance
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0., 0., 0.);

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics, 
            gyroInputs.robotYawRotation2d,
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        wheelsOnlyPoseEstimator = new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            gyroInputs.robotYawRotation2d,
            getModulePositions(), 
            new Pose2d());


        angleController = new PIDController(4.5, 0, 0.0); 
        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(1.5); // degrees, degreesPerSecond.

        translationController = new PIDController(3.75, 0, 0.0); // kP has units of metersPerSecond per meter of error.
        translationController.setTolerance(0.03); // meters, metersPerSecond
        // TODO: real tolerance used to be .02 but changed to .06 for sim

        SmartDashboard.putData("drivetrain/angleController", angleController);
        SmartDashboard.putData("drivetrain/translationController", translationController);

        profiledController = new ProfiledPIDController(2.8, 0, 0.125, new TrapezoidProfile.Constraints(
            4, 3));
        profiledController.setTolerance(0.01, 0.01);
        
        motionProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            4, 3));

        configPathPlanner();
    }

    private void configPathPlanner() {

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = new RobotConfig(0, 0, null);
        }

        AutoBuilder.configure(
            this::getPoseMeters, // Robot pose supplier
            (Pose2d dummy) -> {}, // Method to reset odometry (will be called if your auto has a starting pose)
                                  // Note: We never let PathPlanner set the pose, we always seed pose using cameras and apriltags.
            () -> {return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());}, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds, DriveFeedforwards ff) -> {this.robotOrientedDrive(speeds, true);}, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants // These are different from our angleController gain(s), after testing.
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored
              // We by default draw the paths on the red side of the field, mirroring them if we are on the blue alliance.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

        // Register logging callbacks so that PathPlanner data shows up in advantage scope.
        PathPlannerLogging.setLogActivePathCallback( (activePath) -> {
            Logger.recordOutput("PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

        PathPlannerLogging.setLogTargetPoseCallback( (targetPose) -> {
            // update the desired angle in the angle controller
            // this is only to allow the LEDs to show progress in auto.
            // The actual angle controller that sends commands in auto is the one from PathPlanner.
            double measuredAngleDegrees = getPoseMeters().getRotation().getDegrees();
            double desiredAngleDegrees = targetPose.getRotation().getDegrees();
            angleController.calculate(measuredAngleDegrees, desiredAngleDegrees);
            Logger.recordOutput("PathPlanner/TrajectorySetpoint", targetPose);
        });
    }


    //**************** DRIVING ****************/

    /**
     * Drives the robot based on a desired ChassisSpeeds.
     * <p>
     * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
     * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
     * @param closedLoop - Whether or not to used closed loop PID control to control the speed of the drive wheels.
    */
    public void robotOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds, centerOfRotation_robotFrame.getTranslation());
        setModuleStates(swerveModuleStates, closedLoop);
    }

    /**
     * Drives the robot at a desired chassis speeds. The coordinate system
     * is the same as the one as the one for setPoseMeters().
     * 
     * @param desiredChassisSpeeds - Field relative chassis speeds, in m/s and rad/s. 
     * @param closedLoop - Whether or not to drive the drive wheels with using feedback control.
     */
    public void fieldOrientedDrive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {
        Rotation2d currentOrientation = getPoseMeters().getRotation();
        ChassisSpeeds robotOrientedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredChassisSpeeds, currentOrientation);
        this.robotOrientedDrive(robotOrientedSpeeds, closedLoop);
    }

    private void setModuleStates(SwerveModuleState[] desiredStates, boolean closedLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxAchievableVelocityMetersPerSecond);
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleIndex], closedLoop);
        }
    }


    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            swervePositions[mod.moduleIndex] = mod.getPosition();
        }

        return swervePositions;
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveStates = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            swerveStates[mod.moduleIndex] = mod.getState();
        }

        return swerveStates;
    }

    public void fieldOrientedDriveWhileAiming(ChassisSpeeds desiredTranslationalSpeeds, Rotation2d desiredAngle) {
        // Use PID controller to generate a desired angular velocity based on the desired angle
        double measuredAngle = getPoseMeters().getRotation().getDegrees();
        double desiredAngleDegrees = desiredAngle.getDegrees();
        double desiredDegreesPerSecond = angleController.calculate(measuredAngle, desiredAngleDegrees);
        if (angleController.atSetpoint()) {
            desiredDegreesPerSecond = 0;
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            desiredTranslationalSpeeds.vxMetersPerSecond,
            desiredTranslationalSpeeds.vyMetersPerSecond,
            Units.degreesToRadians(desiredDegreesPerSecond)
        );

        this.fieldOrientedDrive(desiredSpeeds, true);
    }

    public boolean isSideFacingReef() {
        // Find the center of the reef, then get the vector from the robot's
        // current location on the field to the reef.
        Translation2d frontFace = FieldElement.FRONT_REEF_FACE.getLocation2d();
        Translation2d backFace = FieldElement.BACK_REEF_FACE.getLocation2d();
        Translation2d centerOfReef = frontFace.plus(backFace).div(2.0);
        Translation2d robotToReef = centerOfReef.minus(getPoseMeters().getTranslation());

        // Get the direction the robot is pointed in
        Rotation2d robotOrientaion = getPoseMeters().getRotation().plus(Rotation2d.kCCW_90deg.plus(Rotation2d.k180deg));

        // The robot is considered to be facing the reef if the projection
        // of [the robot's orientation] onto [the vector from the robot to the reef]
        // is positive.
        double dotProduct = (robotOrientaion.getCos() * robotToReef.getX()) + (robotOrientaion.getSin() * robotToReef.getY());
        return (dotProduct > 0);
    }

    public boolean isAngleAligned() {
        return angleController.atSetpoint();
    }

    public boolean translationControllerAtSetpoint() {
        return translationController.atSetpoint();
    }

    public void pidToPose(Pose2d desired, double maxSpeedMetersPerSecond) {
        Logger.recordOutput("drivetrain/pidSetpointMeters", desired);

        Pose2d current = getPoseMeters();

        Translation2d error = desired.getTranslation().minus(current.getTranslation());

        Logger.recordOutput("drivetrain/pidErrorMeters", error);
        
        double pidOutputMetersPerSecond = -translationController.calculate(error.getNorm(), 0);


        if (translationController.atSetpoint()) {
            pidOutputMetersPerSecond = 0;
            // System.out.println("true");
        }

        pidOutputMetersPerSecond = MathUtil.clamp(pidOutputMetersPerSecond, -maxSpeedMetersPerSecond, maxSpeedMetersPerSecond);

        Logger.recordOutput("drivetrain/pidOutputMetersPerSecond", pidOutputMetersPerSecond);

        double xMetersPerSecond = pidOutputMetersPerSecond*error.getAngle().getCos();
        double yMetersPerSecond = pidOutputMetersPerSecond*error.getAngle().getSin();
        
        fieldOrientedDriveWhileAiming(
            new ChassisSpeeds(
                xMetersPerSecond,
                yMetersPerSecond,
                0
            ),
            desired.getRotation()
        );
    }

    public void profileToPose(Pose2d desired) {
        Logger.recordOutput("drivetrain/pidSetpointMeters", desired);

        Pose2d current = getPoseMeters();

        Translation2d error = desired.getTranslation().minus(current.getTranslation());

        Logger.recordOutput("drivetrain/pidErrorMeters", error);
        
        double profiledOutputMetersPerSecond = -motionProfile.calculate(0.02, new TrapezoidProfile.State(error.getNorm(), getFieldOrientedVelocity().vxMetersPerSecond + getFieldOrientedVelocity().vyMetersPerSecond),
            new TrapezoidProfile.State(0.0, 0.0)).velocity;


        // copy and pasted tollerance from pid to pose

        double translationPIDoutput = -translationController.calculate(error.getNorm(), 0);

        if (translationController.atSetpoint()) {
            profiledOutputMetersPerSecond = 0;
            translationPIDoutput = 0;
            // System.out.println("AT SETPOINT IN PROFILE TO POSE!!!!!!!!!!");
        }


        double xMetersPerSecond = profiledOutputMetersPerSecond*error.getAngle().getCos();
        double yMetersPerSecond = profiledOutputMetersPerSecond*error.getAngle().getSin();

        double xPIDMetersPerSecond = translationPIDoutput*error.getAngle().getCos();
        double yPIDMetersPerSecond = translationPIDoutput*error.getAngle().getSin();
        
        fieldOrientedDriveWhileAiming(
            new ChassisSpeeds(
                xMetersPerSecond + xPIDMetersPerSecond,
                yMetersPerSecond + yPIDMetersPerSecond,
                0
            ),
            desired.getRotation()
        );
    }



    //**************** ODOMETRY / POSE ESTIMATION ****************/

    /**
     * Sets the current position of the robot on the field in meters.
     * <p>
     * A positive X value brings the robot towards the red alliance,
     * and a positive Y value brings the robot left as viewed by the blue alliance.
     * Rotations are counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * @param pose
     */
    public void setPoseMeters(Pose2d pose) {
        fusedPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
        wheelsOnlyPoseEstimator.resetPosition(gyroInputs.robotYawRotation2d, getModulePositions(), pose);
    }
    public void setOrientation(Rotation2d orientation) {
        // keep location the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(currentPose.getTranslation(), orientation));
    }
    public void setLocation(Translation2d locationOnField) {
        // keep orientation the same
        Pose2d currentPose = getPoseMeters();
        this.setPoseMeters(new Pose2d(locationOnField, currentPose.getRotation()));
    }

    /**
     * Gets the current position of the robot on the field in meters, 
     * based off of our odometry and vision estimation.
     * This value considers the origin to be the right side of the blue alliance.
     * <p>
     * A positive X value brings the robot towards the red alliance, and a positive Y value
     * brings the robot towards the left side as viewed from the blue alliance.
     * <p>
     * Rotations are discontinuous counter-clockwise positive, with an angle of 0 facing away from the blue alliance wall.
     * 
     * @return The current position of the robot on the field in meters.
     */ 
    public Pose2d getPoseMeters() {
        return fusedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the rotation reported by the gyro.
     * This rotation is continuous and counterclockwise positive.
     * 
     * This is not necessarily equivalent to the one reported by getPoseMeters(), and it is recommended
     * to use that rotation in almost every case.
     * 
     * This is usable for calibrating the wheel radii, where a continuous angle is required.
     * @return
     */
    public Rotation2d getGyroRotation2d() {
        return gyroInputs.robotYawRotation2d;
    }

    /**
     * Sets the angle of the robot's pose so that it is facing forward, away from your alliance wall. 
     * This allows the driver to realign the drive direction and other calls to our angle.
     */
    public void setRobotFacingForward() {
        Rotation2d forwardOnRed = Rotation2d.k180deg;
        Rotation2d forwardOnBlue = Rotation2d.kZero;
        Rotation2d forwardNow = getPoseMeters().getRotation();
        this.setOrientation(FlyingCircuitUtils.getAllianceDependentValue(forwardOnRed, forwardOnBlue, forwardNow));
    }


    public void fullyTrustVisionNextPoseUpdate() {
        this.fullyTrustVisionNextPoseUpdate = true;
    }
    public void allowTeleportsNextPoseUpdate() {
        this.allowTeleportsNextPoseUpdate = true;
    }
    public boolean seesAcceptableTag() {
        return this.hasAcceptablePoseObservationsThisLoop;
    }
    private void updatePoseEstimator() {
        // log flags that were set in between last pose update and now
        Logger.recordOutput("drivetrain/fullyTrustingVision", this.fullyTrustVisionNextPoseUpdate);
        Logger.recordOutput("drivetrain/allowingPoseTeleports", this.allowTeleportsNextPoseUpdate);

        // update with wheel deltas
        fusedPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());
        wheelsOnlyPoseEstimator.update(gyroInputs.robotYawRotation2d, getModulePositions());

        Optional <LimelightHelpers.PoseEstimate> mt1Exists = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right"));
        boolean doRejectUpdate = false;
        boolean doesCamExist = true;
        boolean doesLeftCamExist = true;
        LimelightHelpers.PoseEstimate mt1 = new PoseEstimate();
        LimelightHelpers.PoseEstimate mt1LeftCam = new PoseEstimate();

        try {
            mt1Exists.get();
        } catch (Exception NoSuchElementException) {
            doesCamExist = false;
        }
        
        if(doesCamExist) {
            // System.out.println("right Cam exists");
            if(mt1Exists.get() != null) {
                mt1 = mt1Exists.get();
                if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {   
                    if(mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                    }
                    if(mt1.rawFiducials[0].distToCamera > 4) {
                    doRejectUpdate = true;
                    }
                }
            if(mt1.tagCount == 0) {
                doRejectUpdate = true;
            }
            }
        } else {
            doRejectUpdate = true;
        }

        Optional <LimelightHelpers.PoseEstimate> mt1ExistsLeftLimelight = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left"));
        boolean doRejectUpdateFromLeftCam = false;
        // System.out.println(mt1ExistsLeftLimelight.get());
        try {
            mt1ExistsLeftLimelight.get();
        } catch (Exception NoSuchElementException) {
            doesLeftCamExist = false;
        }
        if(doesLeftCamExist) {
            // System.out.println("left Cam exists");
            if(mt1ExistsLeftLimelight.get() != null) {
                    mt1LeftCam = mt1ExistsLeftLimelight.get();
                    if(mt1LeftCam.tagCount == 1 && mt1LeftCam.rawFiducials.length == 1) {   
                        if(mt1LeftCam.rawFiducials[0].ambiguity > .7) {
                            doRejectUpdateFromLeftCam = true;
                        }
                        if(mt1LeftCam.rawFiducials[0].distToCamera > 4) {
                            doRejectUpdateFromLeftCam = true;
                        }
                    }
                    if(mt1LeftCam.tagCount == 0) {
                        doRejectUpdateFromLeftCam = true;
                    }
                    
                }
        } else {
            doRejectUpdateFromLeftCam = true;
        }



        if((doRejectUpdateFromLeftCam && doRejectUpdate) || (!doesCamExist && !doesLeftCamExist)) {
            return;
        }
        if((!doRejectUpdateFromLeftCam && !doRejectUpdate) && (doesCamExist && doesLeftCamExist)) {
            if(mt1.avgTagDist > mt1LeftCam.avgTagDist) {
                mt1 = mt1LeftCam;
            }
        }
        if(doRejectUpdate && !doRejectUpdateFromLeftCam) {
            mt1 = mt1LeftCam;
        }

                        
        Logger.recordOutput("LimelightEstimatedPose", mt1.pose);
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates

        // cam is 0.371475 up and 0.1043 forward in meters


        if(!doRejectUpdate || !doRejectUpdateFromLeftCam) {
            double slopeStdDevMeters_PerMeter = 0.003;
            double tagToCamMeters = mt1.avgTagDist;
            if (tagToCamMeters < 1.5) {
                slopeStdDevMeters_PerMeter = 0.0;
            } else if(tagToCamMeters < 2.5) {
                slopeStdDevMeters_PerMeter = 0.001;
            }
    
            Matrix<N3, N1> stdDevs = this.fullyTrustVisionNextPoseUpdate ? VecBuilder.fill(0, 0, 0) : VecBuilder.fill(
                slopeStdDevMeters_PerMeter*tagToCamMeters, slopeStdDevMeters_PerMeter*tagToCamMeters,9999999);
            fusedPoseEstimator.setVisionMeasurementStdDevs(stdDevs);
            fusedPoseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
        }
    
        else {
            // System.out.println("no mt1");
        }




        // reset flags for next time
        this.fullyTrustVisionNextPoseUpdate = false;
        this.allowTeleportsNextPoseUpdate = false;

    }

    public Translation3d fieldCoordsFromRobotCoords(Translation3d robotCoords) {
        Translation3d robotLocation_fieldFrame = new Translation3d(getPoseMeters().getX(), getPoseMeters().getY(), 0);
        Rotation3d robotOrientation_fieldFrame = new Rotation3d(0, 0, getPoseMeters().getRotation().getRadians());

        return robotCoords.rotateBy(robotOrientation_fieldFrame).plus(robotLocation_fieldFrame);
    }

    public Translation2d fieldCoordsFromRobotCoords(Translation2d robotCoords) {
        return fieldCoordsFromRobotCoords(new Translation3d(robotCoords.getX(), robotCoords.getY(), 0)).toTranslation2d();
    }

    public Translation3d robotCoordsFromFieldCoords(Translation3d fieldCoords) {
        Translation3d robotLocation_fieldFrame = new Translation3d(getPoseMeters().getX(), getPoseMeters().getY(), 0);
        Rotation3d robotOrientation_fieldFrame = new Rotation3d(0, 0, getPoseMeters().getRotation().getRadians());
        Transform3d robotAxesFromFieldPerspective = new Transform3d(robotLocation_fieldFrame, robotOrientation_fieldFrame);
        Transform3d fieldAxesFromRobotPerspecitve = robotAxesFromFieldPerspective.inverse();

        return fieldCoords.rotateBy(fieldAxesFromRobotPerspecitve.getRotation()).plus(fieldAxesFromRobotPerspecitve.getTranslation());
    }

    public Translation2d robotCoordsFromFieldCoords(Translation2d fieldCoords) {
        return robotCoordsFromFieldCoords(new Translation3d(fieldCoords.getX(), fieldCoords.getY(), 0)).toTranslation2d();
    }

    public ChassisSpeeds getFieldOrientedVelocity() {
        ChassisSpeeds robotOrientedSpeeds = DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotOrientedSpeeds, getPoseMeters().getRotation());
    }

    public double getSpeedMetersPerSecond() {
        ChassisSpeeds v = getFieldOrientedVelocity();
        double s = Math.hypot(v.vxMetersPerSecond, v.vyMetersPerSecond);
        return s;
    }

        private FieldElement getClosestFieldElement(FieldElement[] fieldElements) {
        // start by assuming the first is closest
        FieldElement closest = fieldElements[0];
        double minDistance = getPoseMeters().getTranslation().getDistance(closest.getLocation2d());

        // see if any other than the first are closer
        for (int i = 1; i < fieldElements.length; i += 1) {
            FieldElement candidate = fieldElements[i];
            double distance = getPoseMeters().getTranslation().getDistance(candidate.getLocation2d());

            if (distance < minDistance) {
                closest = candidate;
                minDistance = distance;
            }
        }

        return closest;
    }
    public ReefFace getClosestReefFace() {
        return (ReefFace) this.getClosestFieldElement(FieldElement.ALL_REEF_FACES);
    }
    public FieldElement getClosestLoadingStation() {
        return this.getClosestFieldElement(FieldElement.ALL_LOADING_STATIONS);
    }

    public ReefStalk getClosestReefStalk() {
        return (ReefStalk) this.getClosestFieldElement(FieldElement.ALL_STALKS);
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);

        for (SwerveModule mod : swerveModules)
            mod.periodic();

        if (gyroIO instanceof GyroIOSim) //calculates sim gyro
            gyroIO.calculateYaw(getModulePositions());
          

        Logger.processInputs("gyroInputs", gyroInputs);

        updatePoseEstimator();
        // ^^^ intakeCam.periodic() should come after updatePoseEstimator()
        //     so the coral tracking has the most up to date pose info.


        Logger.recordOutput("drivetrain/fusedPose", fusedPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/wheelsOnlyPose", wheelsOnlyPoseEstimator.getEstimatedPosition());
        Logger.recordOutput("drivetrain/speedMetersPerSecond", getSpeedMetersPerSecond());

        Logger.recordOutput("drivetrain/swerveModuleStates", getModuleStates());
        Logger.recordOutput("drivetrain/swerveModulePositions", getModulePositions());

        Translation2d centerOfRotationOnField = getPoseMeters().plus(this.centerOfRotation_robotFrame).getTranslation();
        Logger.recordOutput("drivetrain/centerOfRotationOnField", new Translation3d(centerOfRotationOnField));


    }

}
