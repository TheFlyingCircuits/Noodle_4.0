package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;
    
    ArmFeedforward pivotFeedForward; // in radians

    ProfiledPIDController pivotProfiledPID; // in radians

    public double desiredPivotAngleDegrees;

    Timer timer;

    double lastLoopTime = 0;

    double lastLoopVelocityRadPerSec = 0;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
        timer = new Timer();

        pivotFeedForward = new ArmFeedforward(IntakeConstants.kSPivotVolts, IntakeConstants.kGPivotVolts,IntakeConstants.kVPivotVoltsSecondsPerRadian, IntakeConstants.kAPivotVoltsSecondsSquaredPerRadian);
        pivotProfiledPID = new ProfiledPIDController(IntakeConstants.kPPivotVoltsPerRadian, 0, IntakeConstants.kDPivotVoltsPerRadianPerSecond, 
            new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocityRadianPerSecond, IntakeConstants.maxPivotAccelerationRadianPerSecondSquared));
        timer.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("intakeInputs", inputs);

        goToDesiredPivotAngle();

        Logger.recordOutput("intakeInputs/desiredPivotAngleDegrees", desiredPivotAngleDegrees);
    }

    public void setPivotVolts(double volts) {
        io.setPivotVolts(volts);
    }

    public void setGripperVolts(double topGripperVolts, double bottomGripperVolts) {
        io.setTopGripperVolts(topGripperVolts);
        io.setBottomGripperVolts(bottomGripperVolts);
    }

    public void setPivotTargetAngleDegrees(double targetDegrees) {
        if((targetDegrees > IntakeConstants.maxPivotAngleDegrees) || (targetDegrees < IntakeConstants.minPivotAngleDegrees)) {
            System.out.println("desired angle out of range");
            return;
        }

        desiredPivotAngleDegrees = targetDegrees;
    }

    public void goToDesiredPivotAngle() {
        double profilePIDOutputVolts = pivotProfiledPID.calculate(inputs.pivotAngleRadians, Units.degreesToRadians(desiredPivotAngleDegrees));
        // we use .calculate for that and .setpoint for feed forward because .setpoint is where it should be on the profile but .calculate includes pid and is in volts
        double profileSetpointVelRadPerSec = pivotProfiledPID.getSetpoint().velocity;
        // get acceleration by delta velcocity from thisloop-last loop divided by delta time from thislooptime-lastlooptime
        double accelerationRadPerSecSquared = (profileSetpointVelRadPerSec - lastLoopVelocityRadPerSec) / (timer.get() - lastLoopTime); 
        double feedForwardVolts = pivotFeedForward.calculate(inputs.pivotAngleRadians, profileSetpointVelRadPerSec, accelerationRadPerSecSquared);

        lastLoopTime = timer.get();
        lastLoopVelocityRadPerSec = profileSetpointVelRadPerSec;

        setPivotVolts(profilePIDOutputVolts + feedForwardVolts);
    }

    public Command setTargetAngleDegCommand(double degrees) {
        return this.run(() -> setPivotTargetAngleDegrees(degrees));
    }

    public Command setPivotVoltsCommand(double volts) {
        return this.run(() -> setPivotVolts(volts));
    }

    public Command setGrippersVoltsCommand(double topGripperVolts, double bottomGripperVolts) {
        return this.run(() -> setGripperVolts(topGripperVolts, bottomGripperVolts));
    }
}
