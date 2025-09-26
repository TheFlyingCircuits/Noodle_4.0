package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        public double suckerWheelsRPM = 0.0;
        public double lifterWheelRMP = 0.0;

        public double suckerMotorAppliedCurrent = 0.0;
        public double lifterMotorAppliedCurrent = 0.0;

        public boolean limitSwitchHit = false;
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {};

    public default void setSuckerNeoVolts(double volts) {};

    public default void setLifterNeoVolts(double volts) {};

}

