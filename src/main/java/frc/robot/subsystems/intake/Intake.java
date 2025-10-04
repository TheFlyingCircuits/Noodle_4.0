package frc.robot.subsystems.intake;

import java.util.function.Supplier;

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

    private double desiredPivotAngleDegrees = 90.0;
    private double desiredTopGripperVolts = 0.0;
    private double desiredBottomGripperVolts = 0.0;

    Timer timer;

    double lastLoopTime = 0;

    double lastLoopVelocityRadPerSec = 0;

    boolean hasACoral = false;


    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
        timer = new Timer();

        pivotFeedForward = new ArmFeedforward(IntakeConstants.kSPivotVolts, IntakeConstants.kGPivotVolts,IntakeConstants.kVPivotVoltsSecondsPerRadian, IntakeConstants.kAPivotVoltsSecondsSquaredPerRadian);
        pivotProfiledPID = new ProfiledPIDController(IntakeConstants.kPPivotVoltsPerRadian, 0, IntakeConstants.kDPivotVoltsPerRadianPerSecond, 
            new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocityRadianPerSecond, IntakeConstants.maxPivotAccelerationRadianPerSecondSquared));
        timer.start();
        // pivotProfiledPID.reset(inputs.pivotAngleRadians, 0.0);
    }

    @Override
    public void periodic() {

        inputs.desiredPivotAngleDeg = desiredPivotAngleDegrees;
        inputs.desiredTopGripperVolts = desiredTopGripperVolts;
        inputs.desiredBottomGripperVolts = desiredBottomGripperVolts;
        inputs.hasACoral = hasACoral;

        io.updateInputs(inputs);

        Logger.processInputs("intakeInputs", inputs);

        goToDesiredPivotAngle();
        setGripperVolts(inputs.desiredTopGripperVolts, inputs.desiredBottomGripperVolts);
    }

    public void setPivotVolts(double volts) {
        // System.out.println(volts);
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

    public void setAvePivotAmpsForSim(double amps) {
        inputs.aveTopGripperAmps = amps;
        inputs.aveBottomGripperAmps = amps;
    }

    public void goToDesiredPivotAngle() {
        // System.out.println(-(Units.degreesToRadians(desiredPivotAngleDegrees) - inputs.pivotAngleRadians));

        double profilePIDOutputVolts = pivotProfiledPID.calculate(inputs.pivotAngleRadians,
            new TrapezoidProfile.State(Units.degreesToRadians(inputs.desiredPivotAngleDeg), 0));
        // we use .calculate for that and .setpoint for feed forward because .setpoint is where it should be on the profile but .calculate includes pid and is in volts
        // System.out.println(pivotProfiledPID.getSetpoint().velocity);
        double profileSetpointVelRadPerSec = pivotProfiledPID.getSetpoint().velocity;
        // get acceleration by delta velcocity from thisloop-last loop divided by delta time from thislooptime-lastlooptime
        double accelerationRadPerSecSquared = (profileSetpointVelRadPerSec - lastLoopVelocityRadPerSec) / (timer.get() - lastLoopTime); 
        double feedForwardVolts = pivotFeedForward.calculate(inputs.pivotAngleRadians, profileSetpointVelRadPerSec,accelerationRadPerSecSquared);
        // double feedForwardVolts = pivotFeedForward.calculate(inputs.pivotAngleRadians, profileSetpointVelRadPerSec);


        lastLoopTime = timer.get();
        lastLoopVelocityRadPerSec = profileSetpointVelRadPerSec;

        // System.out.println(inputs.pivotAngleRadians + " rad");

        // System.out.println(feedForwardVolts);

        setPivotVolts(profilePIDOutputVolts + feedForwardVolts);
    }

    public void score(Supplier<Boolean> facingReef) {
        // this function needs to be called in a loop like execute or periodic
        // checks for coral and if we don't have it or we scored it go to defualt 
        if (!inputs.hasACoral) {
            desiredPivotAngleDegrees = IntakeConstants.noCoralPivotSetpointDeg;
            return;
        }
        double scoringPivotDegrees = (facingReef.get()) ? IntakeConstants.frontScorePivotSetpointDeg : IntakeConstants.backScorePivotSetpointDeg; 

    }

    public void defaultFunction() {
        if(hasACoral && (inputs.aveBottomGripperAmps < 10 && inputs.aveTopGripperAmps < 10)) {
            hasACoral = false; // this is for if we drop the coral while doing defualt command the code adjusts by itself
        }
        
        if (hasACoral) {
            desiredPivotAngleDegrees = IntakeConstants.hasCoralPivotSetpointDeg;
            desiredTopGripperVolts = IntakeConstants.holdCoralGripperVolts;
            desiredBottomGripperVolts = IntakeConstants.holdCoralGripperVolts;
            return;
        } else {
            desiredPivotAngleDegrees = IntakeConstants.noCoralPivotSetpointDeg;
            desiredTopGripperVolts = 0.0;
            desiredBottomGripperVolts = 0.0;
        }
    }

    public void intakeCoral() {
        if (inputs.aveBottomGripperAmps > 20 && inputs.aveTopGripperAmps > 20) {
            hasACoral = true;
            defaultFunction();
            return;
        }
            desiredPivotAngleDegrees = IntakeConstants.hasCoralPivotSetpointDeg;
            desiredTopGripperVolts = IntakeConstants.holdCoralGripperVolts;
            desiredBottomGripperVolts = IntakeConstants.holdCoralGripperVolts;
            hasACoral = false;
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
