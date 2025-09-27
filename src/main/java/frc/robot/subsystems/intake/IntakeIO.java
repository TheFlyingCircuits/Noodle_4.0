package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double pivotAngleDegrees = 0;
        public double pivotAngleRadians = 0;
        public double pivotVelocityDegreesPerSecond = 0;
        public double pivotAppliedVolts = 0;
        public double pivotAmps = 0;

        public double topGripperAppliedVolts = 0;
        public double topGripperAmps = 0;

        public double bottomGripperAppliedVolts = 0;
        public double bottomGripperAmps = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setPivotVolts(double volts) {};

    public default void setTopGripperVolts(double volts) {};

    public default void setBottomGripperVolts(double volts) {};

}
