package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class IntakeIONeo implements IntakeIO {
    Neo pivotNeo = new Neo(IntakeConstants.leftPivotNeoID);
    Neo pivotNeoFollow = new Neo(IntakeConstants.rightPivotNeoID);


    Neo topGripperNeo = new Neo(IntakeConstants.topGripperNeoID);
    Neo bottomGripperNeo = new Neo(IntakeConstants.bottomGripperNeoID);

    private SparkMaxConfig pivotConfig;
    private SparkMaxConfig pivotConfigFollower;
    private SparkMaxConfig gipperConfig;

    LinearFilter topGripperCurrentMovingWindow = LinearFilter.singlePoleIIR(0.2, 0.02);
    LinearFilter bottomGripperCurrentMovingWindow = LinearFilter.singlePoleIIR(0.2, 0.02);

    public IntakeIONeo() {
        configMotors();
    }

    private void configMotors() {

        // Pivot Config
        pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.smartCurrentLimit(50);
        pivotConfig.inverted(true); // TODO: set real inversion
        // set to in deg by multipling the 1 rotation by 360 deg/gear ratio, 1*(360/gearRatio)
        
        pivotConfig.encoder.positionConversionFactor(360.0*IntakeConstants.pivotGearReduction)
            .velocityConversionFactor(360.0/60.0*IntakeConstants.pivotGearReduction); // same thing for velocity but bc vel is deg/sec div by 60 seconds

        pivotNeo.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // pivotNeo.getEncoder().setPosition(0.25/IntakeConstants.pivotGearReduction);

        pivotNeo.getEncoder().setPosition(98.5);

        pivotConfigFollower = pivotConfig;

        pivotConfigFollower.follow(IntakeConstants.leftPivotNeoID, true); // make pivotNeoFollower follow pivotNeo

        pivotNeoFollow.configure(pivotConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // apply config to follower

        // Gipper config
        gipperConfig = new SparkMaxConfig();

        gipperConfig.idleMode(IdleMode.kBrake);
        gipperConfig.smartCurrentLimit(50);
        gipperConfig.inverted(true); // TODO: set real inversion

        topGripperNeo.configure(gipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomGripperNeo.configure(gipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        inputs.pivotAngleDegrees = pivotNeo.getEncoder().getPosition();
        SmartDashboard.putNumber("TEST motor position", pivotNeo.getEncoder().getPosition());
        SmartDashboard.putNumber("TEST arm postion", pivotNeo.getEncoder().getPosition() * 360.0*IntakeConstants.pivotGearReduction);

        inputs.pivotAngleRadians = Units.degreesToRadians(pivotNeo.getEncoder().getPosition());
        inputs.pivotFollowerAppliedVolts = pivotNeoFollow.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.pivotVelocityDegreesPerSecond = pivotNeo.getVelocity();
        inputs.pivotAppliedVolts = pivotNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.pivotAmps = pivotNeo.getOutputCurrent();

        inputs.topGripperAppliedVolts = topGripperNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.topGripperAmps = topGripperNeo.getOutputCurrent();
        inputs.aveTopGripperAmps = topGripperCurrentMovingWindow.calculate(inputs.topGripperAmps);

        inputs.bottomGripperAppliedVolts = bottomGripperNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.bottomGripperAmps = bottomGripperNeo.getOutputCurrent();
        inputs.aveBottomGripperAmps = bottomGripperCurrentMovingWindow.calculate(inputs.bottomGripperAmps);
    }

    @Override
    public void setCoastMode(boolean shouldBeInCoast) {
        IdleMode idleMode;
        if (shouldBeInCoast) {
            idleMode = IdleMode.kCoast;
        }
        else{
            idleMode = IdleMode.kBrake;
        }

        pivotConfig.idleMode(idleMode);
        pivotConfigFollower.idleMode(idleMode);


        // pivotNeo.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pivotNeoFollow.configure(pivotConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPivotVolts(double volts) {
        pivotNeo.setVoltage(volts);
    }


    @Override
    public void setTopGripperVolts(double volts) {
        topGripperNeo.setVoltage(volts);
    }

    @Override
    public void setBottomGripperVolts(double volts) {
        bottomGripperNeo.setVoltage(volts);
    }

    @Override
    public void setPivotPosition(double positionDeg){
        pivotNeo.getEncoder().setPosition(positionDeg);
    }
}
