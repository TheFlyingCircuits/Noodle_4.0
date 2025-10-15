package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.ClimberConstants;
import frc.robot.VendorWrappers.Neo;

public class ClimberIONeo implements ClimberIO{
    private Neo sucker;
    private Neo suckerFollow;
    private Neo lifter;
    private Neo lifterFollow;


    private SparkMaxConfig configLifter;
    private SparkMaxConfig configSucker;

    LinearFilter suckerCurrentMovingWindow = LinearFilter.singlePoleIIR(0.2, 0.02);
    LinearFilter suckerFollowerGripperCurrentMovingWindow = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ClimberIONeo() {
        sucker = new Neo(ClimberConstants.suckerNeoID); //TODO put the real canIDs
        lifter = new Neo(ClimberConstants.lifterNeoID); 
        suckerFollow = new Neo(ClimberConstants.suckerFollowerNeoID); //TODO put the real canIDs
        lifterFollow = new Neo(ClimberConstants.lifterFollowerNeoID); 
        configMotors();
    }

    private void configMotors() {

        // lifter moter config
        configLifter = new SparkMaxConfig();
        configLifter.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .inverted(true);
        // configLifter.softLimit.forwardSoftLimitEnabled(false);

        // set the gear reduction
        configLifter.encoder.positionConversionFactor(360*ClimberConstants.lifterGearReduction)
        .velocityConversionFactor(360/60*ClimberConstants.lifterGearReduction);



        // sucker motor config
        configSucker = new SparkMaxConfig();
        configSucker.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(75)
            .inverted(true);
        // configSucker.softLimit.forwardSoftLimitEnabled(false);

        // apply config
        sucker.configure(configSucker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lifter.configure(configLifter, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // change config to follow
        configSucker.follow(ClimberConstants.suckerNeoID, true);
        configLifter.follow(ClimberConstants.lifterNeoID, true);
        
        suckerFollow.configure(configSucker, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lifterFollow.configure(configLifter, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lifter.getEncoder().setPosition(90.0);
        lifterFollow.getEncoder().setPosition(90.0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.suckerWheelsDegPerSec = sucker.getVelocity();
        inputs.suckerFollowerWheelsDegPerSec = suckerFollow.getVelocity();
        inputs.lifterWheeDegPerSec = lifter.getVelocity();
        inputs.lifterFollowerWheeDegPerSec = lifterFollow.getVelocity();

        inputs.suckerMotorAppliedCurrent = sucker.getOutputCurrent();
        inputs.suckerFollowerMotorAppliedCurrent = suckerFollow.getOutputCurrent();
        inputs.lifterMotorAppliedCurrent = lifter.getOutputCurrent();
        inputs.lifterFollowerMotorAppliedCurrent = lifterFollow.getOutputCurrent();

        inputs.lifterAngleDeg = lifter.getPosition();
        inputs.lifterFollowerAngleDeg = lifterFollow.getPosition();

        inputs.suckerAveAmps = suckerCurrentMovingWindow.calculate(inputs.suckerMotorAppliedCurrent);
        inputs.suckerFollowerAveAmps = suckerFollowerGripperCurrentMovingWindow.calculate(inputs.suckerFollowerMotorAppliedCurrent);
        //opGripperNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.suckerAppliedVolts = sucker.getAppliedOutput()*sucker.getBusVoltage();
        inputs.suckerFollowerAppliedVolts = suckerFollow.getAppliedOutput()*suckerFollow.getBusVoltage();
        inputs.lifterMotorAppliedVolts = lifter.getAppliedOutput()*lifter.getBusVoltage();
        inputs.lifterMotorAppliedVolts = lifterFollow.getAppliedOutput()*lifterFollow.getBusVoltage();
    }
    
    @Override
    public void setSuckerNeoVolts(double volts) {
        sucker.setVoltage(volts);
    }

    @Override
    public void setLifterNeoVolts(double volts) {
        lifter.setVoltage(volts);
    }

    public void setLifterPosition(double positionDeg){
        lifter.getEncoder().setPosition(positionDeg);
    }
}
