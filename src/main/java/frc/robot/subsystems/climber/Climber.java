package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

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
        double lifterPIDOutputVolts = climbPID.calculate(inputs.lifterAngleDeg, desiredClimbDegrees);

        setLifterVolts(lifterPIDOutputVolts);
    }

    public void climb() {
        // System.out.println(manualClimbOveride);
        if ((inputs.suckerAveAmps > ClimberConstants.cageDetectedAveAmps || inputs.suckerFollowerAveAmps > ClimberConstants.cageDetectedAveAmps)
        || isClimbing || manualClimbOveride ) {
            setClimbPosition(ClimberConstants.climbingPositionDeg);
            setSuckerVolts(-1);
            isClimbing = true;
            return;
        }

        setLifterPosition(ClimberConstants.aimAtCagePositionDeg);
        setSuckerVolts(-12);

    }

    public Command climbCommand() {
        return this.run(() -> climb());
    }

    public Command setLifterPositionCommand(double desiredLifterDegrees) {
        return this.run(() -> setLifterPosition(desiredLifterDegrees));
    }

    public Command defualtCommand() {
        return this.run(() -> setLifterPosition(90)).alongWith(this.run(() -> setLifterVolts(0)));
    }

}
