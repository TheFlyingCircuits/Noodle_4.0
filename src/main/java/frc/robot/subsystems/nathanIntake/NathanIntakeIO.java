package frc.robot.subsystems.nathanIntake;

import org.littletonrobotics.junction.AutoLog;

public interface NathanIntakeIO {
    @AutoLog
    public class nathanIntakeIOInputs {
        
        public double pivotAngleDegrees = 0;
        public double pivotMotorDPS = 0;
        public double pivotMotorVolts = 0;

        public double gripperMotorVolts = 0;
        public double gripperMotorDPS =0;
        
    }
}
