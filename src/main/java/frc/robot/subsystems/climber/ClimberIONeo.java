package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.VendorWrappers.Neo;

public class ClimberIONeo implements ClimberIO{
        private Neo sucker;
    private Neo lifter;

    private SparkMaxConfig config;

    private ClimberIONeo() {
        sucker = new Neo(0); //TODO put the real canIDs
        lifter = new Neo(0); 

        configMotors();
    }

    private void configMotors() {
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);


        config.softLimit.forwardSoftLimitEnabled(false);

        sucker.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lifter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.suckerWheelsRPM = sucker.getVelocity();
        inputs.lifterWheelRMP = lifter.getVelocity();

        inputs.suckerMotorAppliedCurrent = sucker.getOutputCurrent();
        inputs.lifterMotorAppliedCurrent = lifter.getOutputCurrent();
        
        Logger.recordOutput("climber/sucker/busVolts", sucker.getBusVoltage());
        Logger.recordOutput("climber/sucker/amps", sucker.getOutputCurrent());
        Logger.recordOutput("climber/sucker/dutyCycle", sucker.getAppliedOutput());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/supposedAppliedVolts", sucker.getAppliedOutput() * sucker.getBusVoltage());
        
        Logger.recordOutput("climber/lifter/busVolts", lifter.getBusVoltage());
        Logger.recordOutput("climber/lifter/amps", lifter.getOutputCurrent());
        Logger.recordOutput("climber/lifter/dutyCycle", lifter.getAppliedOutput());
        Logger.recordOutput("climber/lifter/supposedAppliedVolts", lifter.getAppliedOutput() * lifter.getBusVoltage());
       
    }
    
    @Override
    public void setSuckerNeoVolts(double volts) {
        sucker.setVoltage(volts);
    }

    @Override
    public void setLifterNeoVolts(double volts) {
        lifter.setVoltage(volts);
    }
}
