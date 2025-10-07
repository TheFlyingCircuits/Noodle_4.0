package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.VendorWrappers.Neo;

public class Climber extends SubsystemBase {

    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs;
    private PIDController lifterPID;

    private Climber(ClimberIO io) {
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
        io.setSuckerNeoVolts(volts);
    }

    public void setLifterPosition(double desiredLifterDegrees) {
        double lifterPIDOutputVolts = lifterPID.calculate(inputs.lifterAngleDeg, desiredLifterDegrees);

        setLifterVolts(lifterPIDOutputVolts);
    }
}
