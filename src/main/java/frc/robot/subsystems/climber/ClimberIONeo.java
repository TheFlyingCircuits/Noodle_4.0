package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ClimberConstants;
import frc.robot.VendorWrappers.Neo;

public class ClimberIONeo implements ClimberIO{
    private Neo sucker;
    private Neo suckerFollow;
    private Neo lifter;
    private Neo lifterFollow;


    private SparkMaxConfig configLifter;
    private SparkMaxConfig configSucker;

    private ClimberIONeo() {
        sucker = new Neo(0); //TODO put the real canIDs
        lifter = new Neo(0); 
        suckerFollow = new Neo(0); //TODO put the real canIDs
        lifterFollow = new Neo(0); 
        configMotors();
    }

    private void configMotors() {

        // lifter moter config
        configLifter = new SparkMaxConfig();
        configLifter.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);
        configLifter.softLimit.forwardSoftLimitEnabled(false);

        // set the gear reduction
        configLifter.encoder.positionConversionFactor(360*ClimberConstants.lifterGearReduction)
        .velocityConversionFactor(360/60*ClimberConstants.lifterGearReduction);



        // sucker motor config
        configSucker = new SparkMaxConfig();
        configSucker.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);
        configSucker.softLimit.forwardSoftLimitEnabled(false);

        // apply config
        sucker.configure(configSucker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lifter.configure(configLifter, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // change config to follow
        configSucker.follow(0, true);
        configLifter.follow(0);
        
        suckerFollow.configure(configSucker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lifterFollow.configure(configLifter, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.suckerWheelsDegPerSec = sucker.getVelocity();
        inputs.lifterWheeDegPerSec = lifter.getVelocity();

        inputs.suckerMotorAppliedCurrent = sucker.getOutputCurrent();
        inputs.lifterMotorAppliedCurrent = lifter.getOutputCurrent();

        inputs.lifterAngleDeg = lifter.getPosition();
        
        Logger.recordOutput("climber/sucker/busVolts", sucker.getBusVoltage());
        Logger.recordOutput("climber/sucker/amps", sucker.getOutputCurrent());
        Logger.recordOutput("climber/sucker/dutyCycle", sucker.getAppliedOutput());
        Logger.recordOutput("placerGrabber/orangeWheelsNeo/supposedAppliedVolts", sucker.getAppliedOutput() * sucker.getBusVoltage());
        
        Logger.recordOutput("climber/lifter/busVolts", lifter.getBusVoltage());
        Logger.recordOutput("climber/lifter/amps", lifter.getOutputCurrent());
        Logger.recordOutput("climber/lifter/dutyCycle", lifter.getAppliedOutput());
        Logger.recordOutput("climber/lifter/supposedAppliedVolts", lifter.getAppliedOutput() * lifter.getBusVoltage());
        Logger.recordOutput("climber/lifter/positionDeg", lifter.getPosition());
       
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
