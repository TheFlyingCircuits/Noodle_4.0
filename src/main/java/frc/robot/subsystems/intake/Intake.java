package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;
    
    ArmFeedforward pivotFeedForward; // in radians

    ProfiledPIDController pivotProfiledPID; // in radians

    public double desiredPivotAngleDegrees;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();

        pivotFeedForward = new ArmFeedforward(IntakeConstants.kSPivotVolts, IntakeConstants.kGPivotVolts,IntakeConstants.kVPivotVoltsSecondsPerRadian, IntakeConstants.kAPivotVoltsSecondsSquaredPerRadian);
        pivotProfiledPID = new ProfiledPIDController(IntakeConstants.kPPivotVoltsPerRadian, 0, IntakeConstants.kDPivotVoltsPerRadianPerSecond, 
            new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocityRadianPerSecond, IntakeConstants.maxPivotAccelerationRadianPerSecondSquared));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        goToDesiredPivotAngle();

        Logger.recordOutput("intake/desiredPivotAngleDegrees", desiredPivotAngleDegrees);
    }

    public void setPivotVolts(double volts) {
        io.setPivotVolts(volts);
    }

    public void setGripperVolts(double topGripperVolts, double bottomGripperVolts) {
        io.setTopGripperVolts(topGripperVolts);
        io.setBottomGripperVolts(bottomGripperVolts);
    }

    public void setPivotTargetAngleDegrees(double targetDegrees) {
        desiredPivotAngleDegrees = targetDegrees;
    }

    public void goToDesiredPivotAngle() {
        
    }

}
