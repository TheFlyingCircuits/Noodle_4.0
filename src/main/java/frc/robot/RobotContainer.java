// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PlayingField.FieldElement;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.StandardFieldElement;
import frc.robot.commands.L1Score;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIONeo;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIONeo;
import frc.robot.subsystems.intake.IntakeIOSim;


public class RobotContainer {

    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    protected final HumanDriver amara = new HumanDriver(1);
    final CommandXboxController amaraController;


    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Climber climber;
  
    public Pose2d leftSideAutoPathfindingPose;
    public Pose2d rightSideAutoPathfindingPose;
    private boolean manualScoringOveride = false;

    // public SimpleWidget pValueChange;
    public SimpleWidget pivotKpRad;
    public RobotContainer() {

        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
        drivetrain = new Drivetrain( // fr 0.092041015625, br , 0.0419921875, fl -0.178955078125, bl -0.332763671875
            new GyroIOPigeon(),
            new SwerveModuleIONeo(7, 8, -0.184814453125, 0), 
            new SwerveModuleIONeo(5, 6, 0.044677734375, 3),
            new SwerveModuleIONeo(3, 4, -0.3349609375, 2),
            new SwerveModuleIONeo(1, 2,  0.088134765625, 1)
            );

            intake = new Intake(new IntakeIONeo());
            climber = new Climber(new ClimberIONeo());
        }
        else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){}
            );

            intake = new Intake(new IntakeIOSim(drivetrain));
            climber = new Climber(new ClimberIOSim(drivetrain));

        }

        


        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();


        realBindings();
        triggers();

        // pValueChange= Shuffleboard.getTab("robotContainer")
        // .add("pivot pivotDesiredDeg", 0.0);

        pivotKpRad = Shuffleboard.getTab("robotContainer")
        .add("pivot KP", 0.5);

        setDefaultCommands();

        // intake.setDefaultCommand(intake.setTargetAngleDegCommand(90));
    }

    private void realBindings() {

        duncanController.y().onTrue(new InstantCommand(() -> drivetrain.fullyTrustVisionNextPoseUpdate()));

        duncanController.x().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // duncanController.a().whileTrue(Commands.run(() -> intake.setPivotVolts(((pivotSettingVoltage.getEntry().get().getDouble())))));
        // duncanController.a().whileTrue(Commands.run(() -> intake.setPivotTargetAngleDegrees(pivotDesiredDeg.getEntry().get().getDouble())).alongWith(
            // Commands.run(() -> intake.goToDesiredPivotAngle(pivotKpRad.getEntry().get().getDouble()))));

        duncanController.rightTrigger().whileTrue(intake.intakeCommand());

        duncanController.b().whileTrue(Commands.run(() ->manualScoringOveride = true)).whileFalse(Commands.run(() ->manualScoringOveride = false));

        duncanController.leftBumper().whileTrue(intake.ejectCoralCommand());

        duncanController.rightBumper().whileTrue(new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> drivetrain.getClosestReefFace(), () -> duncan.getRequestedFieldOrientedVelocity(),
                () -> drivetrain.getClosestReefStalk() == drivetrain.getClosestReefFace().getLeftStalk(), manualScoringOveride));
        


        // duncanController.leftBumper().whileTrue(climber.setLifterPositionCommand(0)).onFalse(
        //     climber.setLifterPositionCommand(90)
        // );

        // TEST BINDINGS
        // duncanController.rightBumper().whileTrue(Commands.run(() -> intake.setPivotVolts(2)));
        // duncanController.leftBumper().whileTrue(Commands.run(() -> intake.setPivotVolts(-2)));
        
        // duncanController.rightTrigger().whileTrue(Commands.run(() -> climber.setLifterVolts(2)));
        // duncanController.leftTrigger().whileTrue(Commands.run(() -> climber.setLifterVolts(-2)));

        // duncanController.rightTrigger().whileTrue(Commands.run(() -> intake.setPivotVolts(-1)));
            // .onFalse(Commands.run(() -> climber.setSuckerVolts(0)));

        // duncanController.leftTrigger().whileTrue(Commands.run(() -> intake.setPivotVolts(1)));
        //     duncanController.leftBumper().whileTrue(Commands.run(() -> intake.setPivotVolts(0)));
            // .onFalse(Commands.run(() -> climber.setSuckerVolts(0)));

        // duncanController.povUp().whileTrue(Commands.run(() ->  intake.setGripperVolts(2, 2)));
        // duncanController.povDown().whileTrue(Commands.run(() -> intake.setGripperVolts(-2, -2)));

    
        duncanController.povRight().whileTrue(new InstantCommand(() -> climber.setClimbBoolean(false))
            .alongWith(new InstantCommand(() -> climber.setManualClimbBoolean(false))))
        .onFalse(
            climber.climbCommand()
            .alongWith(intake.intakeClimbCommand())
        );

        duncanController.povLeft().whileTrue(Commands.run(() -> climber.setManualClimbBoolean(true)));
        
        duncanController.povUp().whileTrue(climber.setLifterVoltsCommand(-3)).onFalse(climber.homeClimberCommand(90));
    }
    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        // leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
        intake.setDefaultCommand(intake.defaultCommand(pivotKpRad.getEntry().get().getDouble()));
        climber.setDefaultCommand(climber.defualtCommand());
    }


    private void triggers() {
    
    }


    /** Called by Robot.java, convenience function for logging. */
    public void periodic() {

    }    

    private Command driverFullyControlDrivetrain() { return drivetrain.run(() -> {
        drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity(), true);
        Logger.recordOutput("drivetrain/runningDefaultCommand", true);
        }).finallyDo(() -> {
            Logger.recordOutput("drivetrain/runningDefaultCommand", false);
        }).withName("driverFullyControlDrivetrain");
    }

    private Command lineUpWithClosestFace() {
        return Commands.run (() -> drivetrain.pidToPose(drivetrain.getClosestReefFace().getPose2d().plus(new Transform2d(0.5,0,Rotation2d.k180deg)),2));
    }

    /** AUTOS!!!!!!!!!! */

    public Command autoChooser() {

        BooleanSupplier startingOnLeft = () -> {return drivetrain.getClosestLoadingStation() == FieldElement.LEFT_LOADING_STATION;};

        return new ConditionalCommand(leftSideAuto(), rightSideAuto(), startingOnLeft);
    }

    public Command leftSideAuto() {

        //     leftSideAutoPathfindingPose = ReefFace.FRONT_LEFT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()));
        // rightSideAutoPathfindingPose = ReefFace.FRONT_RIGHT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()));
        L1Score scoreOnClosestFace = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> drivetrain.getClosestReefFace(), () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == drivetrain.getClosestReefFace().getLeftStalk(),false);
        L1Score scoreOnFront = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(), false);
            L1Score scoreOnFront2 = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(),false);
        L1Score scoreOnFront3 = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(),false);

        return new SequentialCommandGroup(
            pathfindToPose(ReefFace.FRONT_LEFT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()))).alongWith(intake.defaultCommand(0)).until(
                () -> Math.abs(ReefFace.FRONT_LEFT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d())).minus(drivetrain.getPoseMeters()).getTranslation().getNorm()) < 0.5),

            scoreOnClosestFace.until((scoreOnClosestFace::hasProblablyScored)),

            pickUpLolipop(1,150.0,-0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront.until((scoreOnFront::hasProblablyScored)),

            pickUpLolipop(2,200.0,0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront2.until((scoreOnFront2::hasProblablyScored)),
            
            pickUpLolipop(3,200.0,0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront3.until((scoreOnFront3::hasProblablyScored))
        );
    }

    public Command rightSideAuto() {

        //     leftSideAutoPathfindingPose = ReefFace.FRONT_LEFT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()));
        // rightSideAutoPathfindingPose = ReefFace.FRONT_RIGHT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()));
        L1Score scoreOnClosestFace = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> drivetrain.getClosestReefFace(), () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == drivetrain.getClosestReefFace().getLeftStalk(),false);
        L1Score scoreOnFront = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(),false);
        L1Score scoreOnFront2 = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(),false);
        L1Score scoreOnFront3 = new L1Score(drivetrain,intake, () -> drivetrain.isSideFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds(),() -> drivetrain.getClosestReefStalk() == ReefFace.FRONT_REEF_FACE.getLeftStalk(),false);
            
        return new SequentialCommandGroup(
            pathfindToPose(ReefFace.FRONT_RIGHT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()))).alongWith(intake.defaultCommand(0)).until(
                () -> Math.abs(ReefFace.FRONT_RIGHT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d())).minus(drivetrain.getPoseMeters()).getTranslation().getNorm()) < 0.5),
            scoreOnClosestFace.until((scoreOnClosestFace::hasProblablyScored)),

            pickUpLolipop(3,200.0,0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront.until((scoreOnFront::hasProblablyScored)),

            pickUpLolipop(2,150.0,-0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront2.until((scoreOnFront2::hasProblablyScored)),
            
            pickUpLolipop(1,150.0,-0.25).until(() -> intake.doesHaveACoral()).withTimeout(1.5),
            scoreOnFront3.until((scoreOnFront3::hasProblablyScored))
        );
    }

    public Command pickUpLolipop(int lolipop, double wantedRotationDeg, double adjustedY) { 
        // left it 1 mid is 2 right is 3
        StandardFieldElement lolipopGoingFor;
        double firstX = 0.6;
        double secondX = -0.3;
        // Rotation2d rotation = new Rotation2d();
        if (DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get() == Alliance.Red) {
                adjustedY = -adjustedY;
                firstX = -firstX;
                secondX = -secondX;
                wantedRotationDeg = wantedRotationDeg+ 180;
            }
        }
        if (lolipop == 1) {
            lolipopGoingFor = StandardFieldElement.LEFT_LOLLIPOP;
        } else if (lolipop == 2) {
            lolipopGoingFor = StandardFieldElement.MIDDLE_LOLLIPOP;
        } else if (lolipop == 3) {
            lolipopGoingFor = StandardFieldElement.RIGHT_LOLLIPOP;
        } else {
            return new InstantCommand();
        }
        Pose2d firstPositionToGoTo = new Pose2d (lolipopGoingFor.getPose2d().plus(new Transform2d(firstX, adjustedY, new Rotation2d())).getTranslation(),
            Rotation2d.fromDegrees(wantedRotationDeg));
        Pose2d secondPositionToGoTo = new Pose2d(lolipopGoingFor.getPose2d().plus(new Transform2d(secondX, adjustedY, new Rotation2d())).getTranslation(), 
            Rotation2d.fromDegrees(wantedRotationDeg));
        Logger.recordOutput("first loipop pose", firstPositionToGoTo);

        return new SequentialCommandGroup(
            Commands.run(() -> drivetrain.pidToPose(firstPositionToGoTo, 4))
                .alongWith(intake.intakeCommand()).until(() -> intake.isIntakeDown() && drivetrain.translationControllerAtSetpoint()),
            intake.intakeCommand().alongWith(Commands.run(() -> drivetrain.pidToPose(
                secondPositionToGoTo , 4))).until(() -> intake.doesHaveACoral())
        );
    }

    public Command pathfindToPose(Pose2d targetPose) {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                4.5, 3.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Logger.recordOutput("pathfinding target position", targetPose);
        if (targetPose == null) {
            return new InstantCommand();
        }
        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                1.5
        );
    }
}