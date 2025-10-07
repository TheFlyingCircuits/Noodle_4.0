package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        public double suckerWheelsDegPerSec = 0.0;
        public double lifterWheeDegPerSec = 0.0;

        public double suckerMotorAppliedCurrent = 0.0;
        public double lifterMotorAppliedCurrent = 0.0;

        public double lifterAngleDeg = 180; //TODO make value real
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {};

    public default void setSuckerNeoVolts(double volts) {};

    public default void setLifterNeoVolts(double volts) {};

}

