package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class IntakeIONeo implements IntakeIO {
    Neo leftPivotNeo = new Neo(IntakeConstants.leftPivotNeoID);
    Neo rightPivotNeo = new Neo(IntakeConstants.rightPivotNeoID);

    Neo leftGripperNeo = new Neo(IntakeConstants.leftGripperNeoID);
    Neo rightGripperNeo = new Neo(IntakeConstants.rightGripperNeoID);

    private SparkMaxConfig config;


    private void configMotors() {

        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(true);

        config.softLimit.forwardSoftLimitEnabled(false);

        leftPivotNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //wristNeo.setPosition(WristConstants.maxAngleDegrees);
    }
    
}
