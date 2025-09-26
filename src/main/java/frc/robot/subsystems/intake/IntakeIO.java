package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double pivotAngleDegrees = 0;
        public double pivotAngleDegreesPerSecond = 0;
        public double pivotAppliedVolts = 0;
        public double pivotAmps = 0;

        public double gripperAppliedVolts = 0;
        public double gripperAmps = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setPivotVolts(double volts) {};

    public default void setTargetAngleDegrees(double angleDegrees) {};

    public default void setGripperVolts(double volts) {};





}
