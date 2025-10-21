package frc.robot.subsystems.nathanIntake;

import org.littletonrobotics.junction.AutoLog;

public interface NathanIntakeIO {
    @AutoLog
    public class nathanIntakeIOInputs {
        
        public double pivotAngleDegrees = 0;
        public double pivotMotorDPS = 0;
        public double pivotMotorVolts = 0;
        public double pivotDesiredDeg = 0;

        public double gripperTopMotorVolts = 0;
        public double gripperTopMotorDPS =0;
        public double gripperTopDesierdVolts = 0;

        public double gripperBottomMotorVolts = 0;
        public double gripperBottomMotorDPS =0;
        public double gripperBottomDesierdVolts = 0;
    }

    public default void updateInputs(nathanIntakeIOInputs inputs) {};
    public default void setGripperVolts(double tVolts, double bVolts ) {};
    public default void setPivotVolts(double volts) {};
    public default void setPivotPosition(double position) {};
}
