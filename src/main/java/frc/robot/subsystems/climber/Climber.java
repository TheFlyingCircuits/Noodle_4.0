package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs;
    private PIDController lifterPID;

    public Climber(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();

        lifterPID = new PIDController(ClimberConstants.kPLifterVolts, 0, 0);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    } 

    public void setSuckerVolts(double volts) {
        io.setSuckerNeoVolts(volts);
    }

    public void setLifterVolts(double volts) {
        io.setLifterNeoVolts(volts);
    }

    public void setLifterPosition(double desiredLifterDegrees) {
        double lifterPIDOutputVolts = lifterPID.calculate(inputs.lifterAngleDeg, desiredLifterDegrees);

        setLifterVolts(lifterPIDOutputVolts);
    }

    public void climb() {
        if (inputs.suckerMotorAppliedCurrent > 20) {
            setLifterPosition(ClimberConstants.climbingPositionDeg);
            setSuckerVolts(-1);
        }

        setSuckerVolts(-12);

    }

    public Command setLifterPositionCommand(double desiredLifterDegrees) {
        return this.run(() -> setLifterPosition(desiredLifterDegrees));
    }
}
