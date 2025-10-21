package frc.robot.subsystems.nathanIntake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.VendorWrappers.Neo;

public class NathanIntakeIONeo implements NathanIntakeIO{

    Neo pivotNeo = new Neo(IntakeConstants.leftPivotNeoID);
    Neo pivotNeoFollow = new Neo(IntakeConstants.rightPivotNeoID);
    Neo bottomGripperNeo = new Neo(IntakeConstants.bottomGripperNeoID);
    Neo topGripperNeo = new Neo(IntakeConstants.topGripperNeoID);

    SparkMaxConfig pivotConfig;
    SparkMaxConfig gripperConfig;

    public void configMotors() {
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


        pivotConfig.follow(IntakeConstants.leftPivotNeoID, true); // make pivotNeoFollower follow pivotNeo

        pivotNeoFollow.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); // apply config to follower

        // Gipper config
        gripperConfig = new SparkMaxConfig();

        gripperConfig.idleMode(IdleMode.kBrake);
        gripperConfig.smartCurrentLimit(50);
        gripperConfig.inverted(true); // TODO: set real inversion

        topGripperNeo.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomGripperNeo.configure(gripperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

}