package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.PlayingField.FieldConstants;
import frc.robot.PlayingField.ReefFace;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

public class L1Score extends Command {
    private Drivetrain drivetrain;
    private Intake intake;
    private boolean ifFacingReef;
    private ReefFace faceScoringOn;
    private ChassisSpeeds driverRequestedVel;

    private Pose2d adjustedReefScoringPose(ReefFace face, boolean isFacingForward, ChassisSpeeds overideY) {
        double adjustedX = FieldConstants.stalkInsetMeters;        // puts center of robot at the outer edge of the reef
        adjustedX += DrivetrainConstants.bumperWidthMeters / 2.0;  // move back a half bumper length so the bumper is touching the edge of the reef
        adjustedX += 2; // measure this manually

        Pose2d targetPose = face.getPose2d();

        double adjustedY = 0;


        if (face == ReefFace.FRONT_REEF_FACE || face == ReefFace.FRONT_LEFT_REEF_FACE || face == ReefFace.FRONT_RIGHT_REEF_FACE) {

        }

        
        Rotation2d rotationAdjustment;
        if (isFacingForward) {
            rotationAdjustment = Rotation2d.k180deg;
        } else {
            rotationAdjustment = Rotation2d.kZero;
        }



        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotationAdjustment);
        Pose2d scoringPose = targetPose.plus(targetPoseToRobotRelativeToStalk);
        Logger.recordOutput("L1Scoring/targetDrivePose", scoringPose);
        return scoringPose;
    }
}



