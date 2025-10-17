package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double pivotAngleDegrees = -19.0;
        public double pivotAngleRadians = Units.degreesToRadians(-19.0);
        public double pivotVelocityDegreesPerSecond = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotFollowerAppliedVolts = 0.0;;
        public double pivotAmps = 0.0;
        public double desiredPivotAngleDeg = -19.0;

        public double desiredTopGripperVolts = 0.0;
        public double topGripperAppliedVolts = 0.0;
        public double topGripperAmps = 0.0;
        public double aveTopGripperAmps = 0.0;

        public double desiredBottomGripperVolts = 0.0;
        public double bottomGripperAppliedVolts = 0.0;
        public double bottomGripperAmps = 0.0;
        public double aveBottomGripperAmps = 0.0;

        public boolean hasACoral = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {};

    public default void setPivotVolts(double volts) {};

    public default void setPivotPosition(double positionDegrees) {};

    public default void setCoastMode(boolean shouldBeInCoast) {};

    public default void setTopGripperVolts(double volts) {};

    public default void setBottomGripperVolts(double volts) {};

}
