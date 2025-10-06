package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public L1Score(Drivetrain drivetrain, Intake intake, Supplier<Boolean> ifFacingReef, Supplier<ReefFace> faceScoringOn, Supplier<ChassisSpeeds> driverRequestedVel) {
        this.drivetrain=drivetrain;
        this.intake=intake;
        this.ifFacingReef=ifFacingReef;
        this.faceScoringOn=faceScoringOn;
        this.driverRequestedVel=driverRequestedVel;
        scoringTimer = new Timer();
    }

    private Pose2d adjustedReefScoringPose(ReefFace face, boolean isFacingForward, ChassisSpeeds overideY) {
        // double adjustedX = FieldConstants.stalkInsetMeters;        // puts center of robot at the outer edge of the reef
        double adjustedX = DrivetrainConstants.bumperWidthMeters / 2.0;  // move back a half bumper length so the bumper is touching the edge of the reef
        adjustedX += 0.2; // measure this manually

        Pose2d targetPose = face.getPose2d();

        double adjustedY = targetPose.minus(drivetrain.getPoseMeters()).getY();

        if (!isFacingForward) {
            adjustedY = -adjustedY;
        }
        adjustedY = MathUtil.clamp(adjustedY, -0.4, 0.4);
        // double adjustedY = 0;


        if (face == ReefFace.FRONT_REEF_FACE || face == ReefFace.FRONT_LEFT_REEF_FACE || face == ReefFace.FRONT_RIGHT_REEF_FACE) {
            adjustedY += -overideY.vyMetersPerSecond / 12;
        } else {
            adjustedY += overideY.vyMetersPerSecond / 12;
        }


        adjustedY = MathUtil.clamp(adjustedY, -0.4, 0.4);

        
        Rotation2d rotationAdjustment;
        if (isFacingForward) {
            rotationAdjustment = Rotation2d.k180deg;
        } else {
            rotationAdjustment = Rotation2d.kZero;
        }



        Transform2d targetPoseToRobotRelativeToStalk = new Transform2d(adjustedX, adjustedY, rotationAdjustment);
        Pose2d scoringPose = targetPose.plus(targetPoseToRobotRelativeToStalk);
        // Logger.recordOutput("L1Scoring/targetDrivePose", scoringPose);
        return scoringPose;
    }

    public boolean hasProblablyScored() {
        System.out.println(scoringTimer.get() > 2);
        return scoringTimer.get() > 2;
    }

    @Override
    public void execute() {
        // System.out.println(faceScoringOn.get().getName());
        Pose2d adjustedPose = adjustedReefScoringPose(faceScoringOn.get(), ifFacingReef.get(), driverRequestedVel.get());
        Logger.recordOutput("L1Scoring/targetDrivePose", adjustedPose);
        drivetrain.pidToPose(adjustedPose, 3.0);

        boolean readyToScore = drivetrain.translationControllerAtSetpoint() && drivetrain.isAngleAligned();
        intake.score(ifFacingReef, readyToScore);
        
        if(readyToScore && timerHasNotStarted) {
            scoringTimer.start();
            timerHasNotStarted = false;
        }

        if(!readyToScore) {
            scoringTimer.stop();
            scoringTimer.reset();
            timerHasNotStarted = true;
        }

        
    }
}



