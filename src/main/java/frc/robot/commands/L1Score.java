package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PlayingField.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

public class L1Score extends Command {
    private Drivetrain drivetrain;
    private Intake intake;
    private Supplier<ChassisSpeeds> driverRequestedVel;
    private Supplier<ReefFace> faceScoringOn;
    private Supplier<Boolean> ifFacingReef;
    private Timer scoringTimer;
    private boolean timerHasNotStarted = true;
    private boolean isGoingForLeft;
    private Supplier<Boolean> manualScoreOveride;
    

    public L1Score(Drivetrain drivetrain, Intake intake, Supplier<Boolean> ifFacingReef, Supplier<ReefFace> faceScoringOn, 
        Supplier<ChassisSpeeds> driverRequestedVel, Supplier<Boolean> isClosestStalkLeft, Supplier<Boolean> manualScoreOveride) {
        this.drivetrain=drivetrain;
        this.intake=intake;
        this.ifFacingReef=ifFacingReef;
        this.faceScoringOn=faceScoringOn;
        this.driverRequestedVel=driverRequestedVel;
        this.manualScoreOveride=manualScoreOveride;
        scoringTimer = new Timer();
        isGoingForLeft = isClosestStalkLeft.get();
        addRequirements(drivetrain, intake);
    }

    private Pose2d adjustedReefScoringPose(ReefFace face, boolean isFacingForward, ChassisSpeeds overideY) {
        // double adjustedX = FieldConstants.stalkInsetMeters;        // puts center of robot at the outer edge of the reef
        double adjustedX = DrivetrainConstants.bumperWidthMeters / 2.0;  // move back a half bumper length so the bumper is touching the edge of the reef
        if(isFacingForward){
            adjustedX += 0.125; // measure this manually
        } else {
            adjustedX += -0.06;
        }

        double adjustedY;
        boolean noChangeInY = overideY.vyMetersPerSecond == 0;
        boolean isYPositive = !(overideY.vyMetersPerSecond >= 0);// inverse bc needs to be flipped around for left on joystick to move to the left setpoint

        if (face == ReefFace.BACK_REEF_FACE || face == ReefFace.BACK_LEFT_REEF_FACE || face == ReefFace.BACK_RIGHT_REEF_FACE) {
            isYPositive = !isYPositive;
        }
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            isYPositive = !isYPositive;
        }


        Pose2d targetPose = face.getPose2d();

        // Translation2d vectorRobotToReef = targetPose.getTranslation().minus(drivetrain.getPoseMeters().getTranslation());

        // double adjustedY = face.getPose2d().getRotation().minus(
        //     vectorRobotToReef.getAngle().plus(Rotation2d.k180deg)).getDegrees();
        

        // adjustedY = adjustedY/85.0;

        // double adjustedY = lastAdjustedY;
        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     adjustedY = -adjustedY;
        // }

        // adjustedY = MathUtil.clamp(adjustedY, -0.4, 0.4);
        // double adjustedY = 0;


        // if (face == ReefFace.FRONT_REEF_FACE || face == ReefFace.FRONT_LEFT_REEF_FACE || face == ReefFace.FRONT_RIGHT_REEF_FACE) {
        //     adjustedY += overideY.vyMetersPerSecond / 12.0;
        // } else {
        //     adjustedY += -overideY.vyMetersPerSecond / 12.0;
        // }


        // adjustedY = MathUtil.clamp(adjustedY, -0.4, 0.4);
        // System.out.println(adjustedY);

        if(!noChangeInY) {
            isGoingForLeft = isYPositive;
        }

        double centerToIntakeMeters = 0.12065;
        if(isFacingForward) { // .25 each way +- .125
            adjustedY = isGoingForLeft? 0.25 - centerToIntakeMeters : -0.25 - centerToIntakeMeters;
        } else{
            adjustedY = isGoingForLeft? 0.25 + centerToIntakeMeters : -0.25 + centerToIntakeMeters;
        }
        
        Rotation2d rotationAdjustment;
        if (isFacingForward) {
            // System.out.println("facing reef");
            // rotationAdjustment = Rotation2d.k180deg;
            rotationAdjustment = new Rotation2d().minus(Rotation2d.kCW_90deg.plus(Rotation2d.k180deg));
            // rotationAdjustment = new Rotation2d();
        } else {
            // System.out.println("Not");
            // rotationAdjustment = Rotation2d.kZero;
            rotationAdjustment = new Rotation2d().minus(Rotation2d.kCW_90deg);
        }



        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, new Rotation2d());
        Pose2d scoringPose = targetPose.plus(targetPoseToRobotRelativeToStalk);
        scoringPose = new Pose2d(scoringPose.getTranslation(), scoringPose.getRotation().plus(rotationAdjustment));
        // Logger.recordOutput("L1Scoring/targetDrivePose", scoringPose);
        return scoringPose;
    }

    public boolean hasProblablyScored() {
        // System.out.println(scoringTimer.get() > 0.5);
        return scoringTimer.get() > 0.75;
    }

    @Override
    public void execute() {
        drivetrain.fullyTrustVisionNextPoseUpdate();
        // System.out.println(faceScoringOn.get().getName());
        Pose2d adjustedPose = adjustedReefScoringPose(faceScoringOn.get(), ifFacingReef.get(), driverRequestedVel.get());
        Logger.recordOutput("L1Scoring/targetDrivePose", adjustedPose);

        boolean closeToScoringPose = adjustedPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 1;

        boolean basicallyAtScoringSetpoint = adjustedPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm() < 0.06;
        
        if (!closeToScoringPose) {
            // drivetrain.profileToPose(adjustedPose);
            drivetrain.pidToPose(adjustedPose, 2.5);
        } else {
            drivetrain.pidToPose(adjustedPose, 1.5);
        }
        // drivetrain.profileToPose(adjustedPose);
        // drivetrain.pidToPose(adjustedPose, 2.0);

        boolean readyToScore = (drivetrain.translationControllerAtSetpoint() || (DriverStation.isAutonomous() && basicallyAtScoringSetpoint)) && drivetrain.isAngleAligned();
        Logger.recordOutput("L1Scoring/readyToScore", readyToScore);
        Logger.recordOutput("L1Scoring/isAngleAligned", drivetrain.isAngleAligned());
        Logger.recordOutput("L1Scoring/translationControllerAtSetpoint", drivetrain.translationControllerAtSetpoint());
        if(manualScoreOveride.get()){
            Logger.recordOutput("L1Scoring/manualOveride", true);
            readyToScore= true;
        } else {
            Logger.recordOutput("L1Scoring/manualOveride", false);
        }
        // System.out.println(readyToScore);
        intake.score(ifFacingReef, readyToScore, manualScoreOveride.get());
        
        if(readyToScore && timerHasNotStarted) {
            scoringTimer.start();
            timerHasNotStarted = false;
        }

        // if(!readyToScore) {
        //     scoringTimer.stop();
        //     scoringTimer.reset();
        //     timerHasNotStarted = true;
        // }

        
    }
}



