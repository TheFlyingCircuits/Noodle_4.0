package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
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

        pivotConfig.idleMode(IdleMode.kBrake);
        gipperConfig.smartCurrentLimit(30);
        gipperConfig.inverted(true); // TODO: set real inversion
        // set to in deg by multipling the 1 rotation by 360 deg/gear ratio, 1*(360/gearRatio)
        
        pivotConfig.encoder.positionConversionFactor(360/IntakeConstants.pivotGearReduction)
            .velocityConversionFactor(360/60/IntakeConstants.pivotGearReduction); // same thing for velocity but bc vel is deg/sec div by 60 seconds

        pivotNeo.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Gipper config
        gipperConfig = new SparkMaxConfig();

        gipperConfig.idleMode(IdleMode.kBrake);
        gipperConfig.smartCurrentLimit(30);
        gipperConfig.inverted(true); // TODO: set real inversion

        topGripperNeo.configure(gipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomGripperNeo.configure(gipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotAngleDegrees = pivotNeo.getPosition();
        inputs.pivotAngleRadians = Units.degreesToRadians(inputs.pivotAngleDegrees);
        inputs.pivotVelocityDegreesPerSecond  = pivotNeo.getVelocity();
        inputs.pivotAppliedVolts = pivotNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.pivotAmps = pivotNeo.getOutputCurrent();

        inputs.topGripperAppliedVolts = topGripperNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.topGripperAmps = topGripperNeo.getOutputCurrent();

        inputs.bottomGripperAppliedVolts = bottomGripperNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.bottomGripperAmps = bottomGripperNeo.getOutputCurrent();
    }

    @Override
    public void setPivotVolts(double volts) {
        pivotNeo.setVoltage(volts);
    }
        
    public void setTopGripperVolts(double volts) {
        topGripperNeo.setVoltage(volts);
    }

    public void setBottomGripperVolts(double volts) {
        bottomGripperNeo.setVoltage(volts);
    }
}
