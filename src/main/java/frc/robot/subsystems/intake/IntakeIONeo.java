package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class IntakeIONeo implements IntakeIO {
    Neo pivotNeo = new Neo(IntakeConstants.leftPivotNeoID);

    Neo topGripperNeo = new Neo(IntakeConstants.leftGripperNeoID);
    Neo bottomGripperNeo = new Neo(IntakeConstants.rightGripperNeoID);

    private SparkMaxConfig pivotConfig;
    private SparkMaxConfig gipperConfig;

    public IntakeIONeo() {
        configMotors();
    }


    private void configMotors() {


        // Pivot Config
        pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(true); // TODO: set real inversion
        // set to in deg by multipling the 1 rotation by 360 deg/gear ratio, 1*(360/gearRatio)
        pivotConfig.encoder.positionConversionFactor(360/IntakeConstants.pivotGearRatio)
            .velocityConversionFactor(360/60/IntakeConstants.pivotGearRatio); // same thing for velocity but bc vel is deg/sec div by 60 seconds

        pivotConfig.softLimit.forwardSoftLimitEnabled(false);

        pivotNeo.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Gipper config
        gipperConfig = new SparkMaxConfig();

        gipperConfig.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(true); // TODO: set real inversion

        //wristNeo.setPosition(WristConstants.maxAngleDegrees);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotAngleDegrees = pivotNeo.getPosition();
        inputs.pivotVelocityDegreesPerSecond  = pivotNeo.getVelocity();
        inputs.pivotAppliedVolts = pivotNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.pivotAmps = pivotNeo.getOutputCurrent();




    }
    
}
