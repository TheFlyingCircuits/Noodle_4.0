package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs;
    private PIDController lifterPID;
    private PIDController climbPID;
    private boolean isClimbing = false;
    private boolean manualClimbOveride = false;

    public Climber(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();

        lifterPID = new PIDController(ClimberConstants.kPLifterVolts, 0, 0);
        climbPID = new PIDController(ClimberConstants.kPClimbingLifterVoltsPerDegree, 0, ClimberConstants.kDClimbingLifterVoltsPerDegreePerSec);


    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("climberInputs", inputs);
    } 

    public void setSuckerVolts(double volts) {
        io.setSuckerNeoVolts(volts);
    }

    public void setLifterVolts(double volts) {
        io.setLifterNeoVolts(volts);
    }

    public void setBoth(double liftPos, double suckerVolts) {
        setLifterPosition(liftPos);
        io.setSuckerNeoVolts(suckerVolts);
    }

    public void setClimbBoolean(boolean climbing) {
        isClimbing = climbing;
    }

    public void setManualClimbBoolean(boolean manualClimb) {
        manualClimbOveride = manualClimb;
    }

    public void setLifterPosition(double desiredLifterDegrees) {
        double lifterPIDOutputVolts = lifterPID.calculate(inputs.lifterAngleDeg, desiredLifterDegrees);

        setLifterVolts(lifterPIDOutputVolts);
    }

    public void setClimbPosition(double desiredClimbDegrees) {
        double voltageOutput;
        if(80 < inputs.lifterAngleDeg && inputs.lifterAngleDeg < 110){
            voltageOutput = MathUtil.clamp(climbPID.calculate(inputs.lifterAngleDeg, desiredClimbDegrees)
            ,-1.0,4.0);
        } else {
            // voltageOutput = climbPID.calculate(inputs.lifterAngleDeg, desiredClimbDegrees);
            voltageOutput = MathUtil.clamp(climbPID.calculate(inputs.lifterAngleDeg, desiredClimbDegrees)
            ,-1.0,4.0);
        }
        // voltageOutput = climbPID.calculate(inputs.lifterAngleDeg, desiredClimbDegrees);

        setLifterVolts(voltageOutput);
    }

    public void climb() {
        // System.out.println(manualClimbOveride);
        if ((inputs.suckerAveAmps > ClimberConstants.cageDetectedAveAmps || inputs.suckerFollowerAveAmps > ClimberConstants.cageDetectedAveAmps)
        || isClimbing || manualClimbOveride ) {
            setClimbPosition(ClimberConstants.climbingPositionDeg);
            setSuckerVolts(-2);
            isClimbing = true;
            return;
        }

        setLifterPosition(ClimberConstants.aimAtCagePositionDeg);
        setSuckerVolts(-10);

    }

    public void homeClimber(double positionDeg){
        // sets the Actuall position for homing
        io.setLifterNeoVolts(0);
        io.setLifterPosition(positionDeg);
    }

    public Command setLifterVoltsCommand(double volts) {
        return this.run(() -> setLifterVolts(volts));
    }

    public Command homeClimberCommand(double angleDeg) {
        return this.run(() -> homeClimber(angleDeg));
    }

    public Command climbCommand() {
        return this.run(() -> climb());
    }

    public Command setLifterPositionCommand(double desiredLifterDegrees) {
        return this.run(() -> setLifterPosition(desiredLifterDegrees));
    }

    public Command defualtCommand() {
        return this.run(() -> setBoth(90.0,0.0));
    }

}
