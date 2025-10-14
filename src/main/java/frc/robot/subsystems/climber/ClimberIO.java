package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.ClimberConstants;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        public double suckerWheelsDegPerSec = 0.0;
        public double suckerFollowerWheelsDegPerSec = 0.0;
        public double lifterWheeDegPerSec = 0.0;
        public double lifterFollowerWheeDegPerSec = 0.0;

        public double suckerMotorAppliedCurrent = 0.0;
        public double suckerFollowerMotorAppliedCurrent = 0.0;
        public double lifterMotorAppliedCurrent = 0.0;
        public double lifterFollowerMotorAppliedCurrent = 0.0;

        public double suckerAppliedVolts = 0.0;
        public double suckerFollowerAppliedVolts = 0.0;
        public double lifterMotorAppliedVolts = 0.0;
        public double lifterFollowerMotorAppliedVolts = 0.0;

        public double suckerAveAmps = 0;
        public double suckerFollowerAveAmps;

        public double lifterAngleDeg = ClimberConstants.climberStartingPositionDeg; //TODO make value real
        public double lifterFollowerAngleDeg = ClimberConstants.climberStartingPositionDeg;
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {};

    public default void setSuckerNeoVolts(double volts) {};

    public default void setLifterNeoVolts(double volts) {};

    public default void setLifterPosition(double positionDeg){};

}

