// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PlayingField.ReefFace;
import frc.robot.PlayingField.StandardFieldElement;
import frc.robot.commands.L1Score;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIONeo;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;


public class RobotContainer {

    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    protected final HumanDriver amara = new HumanDriver(1);
    final CommandXboxController amaraController;


    public final Drivetrain drivetrain;
    public final Leds leds;
    public final Intake intake;
  
    public Pose2d leftSideAutoPathfindingPose;
    public Pose2d rightSideAutoPathfindingPose;
    
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

            leds = new Leds();
            intake = new Intake(new IntakeIOSim(drivetrain));
        }
        else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){}
            );

            leds = new Leds();
            intake = new Intake(new IntakeIOSim(drivetrain));

        }

        setDefaultCommands();
        


        duncanController = duncan.getXboxController();
        amaraController = amara.getXboxController();


        leftSideAutoPathfindingPose = ReefFace.FRONT_LEFT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, new Rotation2d()));
        rightSideAutoPathfindingPose = ReefFace.FRONT_RIGHT_REEF_FACE.getPose2d().plus(new Transform2d(1.5,0, Rotation2d.kZero));

        realBindings();
        triggers();

        // intake.setDefaultCommand(intake.setTargetAngleDegCommand(90));
    }

    private void realBindings() {

        duncanController.y().onTrue(new InstantCommand(() -> drivetrain.fullyTrustVisionNextPoseUpdate()));
        duncanController.povUp().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));

        // duncanController.rightBumper().whileTrue(lineUpWithClosestFace());

        // duncanController.x().whileTrue(intake.setTargetAngleDegCommand(0.0)).whileFalse(intake.setTargetAngleDegCommand(90.0));

        // duncanController.b().whileTrue(intake.setPivotVoltsCommand(2));

        duncanController.rightTrigger().whileTrue(intake.intakeCommand());

        duncanController.leftBumper().whileTrue(new L1Score(drivetrain,intake, () -> drivetrain.isFacingReef(), 
            () -> drivetrain.getClosestReefFace(), () -> duncan.getRequestedFieldOrientedVelocity()));

        if(RobotBase.isSimulation()) {
            duncanController.a().whileTrue(Commands.run(() -> intake.setAvePivotAmpsForSim(25)));
            duncanController.b().whileTrue(Commands.run(() -> intake.setAvePivotAmpsForSim(0)));
            duncanController.x().whileTrue(Commands.run(() -> Logger.recordOutput("coral + x",
                StandardFieldElement.LEFT_LOLLIPOP.getPose2d().plus(new Transform2d(1,0, new Rotation2d())))));
        }

    }
    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
        intake.setDefaultCommand(intake.defaultCommand());
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
        return leftSideAuto();
    }

    public Command leftSideAuto() {
        L1Score scoreOnClosestFace = new L1Score(drivetrain,intake, () -> drivetrain.isFacingReef(), 
            () -> drivetrain.getClosestReefFace(), () -> new ChassisSpeeds());
        L1Score scoreOnFront = new L1Score(drivetrain,intake, () -> drivetrain.isFacingReef(), 
            () -> ReefFace.FRONT_REEF_FACE, () -> new ChassisSpeeds());
        return new SequentialCommandGroup(
            pathfindToPose(leftSideAutoPathfindingPose).alongWith(intake.defaultCommand()).until(
                () -> Math.abs(leftSideAutoPathfindingPose.minus(drivetrain.getPoseMeters()).getTranslation().getNorm()) < 0.5),

            scoreOnClosestFace.until((scoreOnClosestFace::hasProblablyScored)),
            pickUpLolipop(3).until(() -> intake.doesHaveACoral()).withTimeout(3),
            scoreOnFront.until((scoreOnFront::hasProblablyScored))
        );
    }

    public Command pickUpLolipop(double lolipop) { 
        // left it 1 mid is 2 right is 3
        StandardFieldElement lolipopGoingFor;

        if (lolipop == 1) {
            lolipopGoingFor = StandardFieldElement.LEFT_LOLLIPOP;
        } else if (lolipop == 2) {
            lolipopGoingFor = StandardFieldElement.MIDDLE_LOLLIPOP;
        } else if (lolipop == 3) {
            lolipopGoingFor = StandardFieldElement.RIGHT_LOLLIPOP;
        } else {
            return new InstantCommand();
        }
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                drivetrain.getPoseMeters(),
                new Pose2d (lolipopGoingFor.getPose2d().plus(new Transform2d(1,0, new Rotation2d())).getTranslation(),
                    Rotation2d.fromDegrees(135))
        );

        PathConstraints constraints = new PathConstraints(0.5, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(135)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return new SequentialCommandGroup(
            AutoBuilder.followPath(path).alongWith(intake.intakeCommand()).until(() -> intake.isIntakeDown()),
            intake.intakeCommand().alongWith(Commands.run(() -> drivetrain.pidToPose(
                new Pose2d(lolipopGoingFor.getPose2d().plus(new Transform2d(-1,0, new Rotation2d())).getTranslation(), 
                    Rotation2d.fromDegrees(135)) , 0.5))).until(() -> intake.doesHaveACoral())
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