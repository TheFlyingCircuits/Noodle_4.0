package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class IntakeIOSim implements IntakeIO{
    
    private FlywheelSim pivotSim;
    private Drivetrain drivetrain;

    public IntakeIOSim(Drivetrain drivetrain) {
        // fake values for sim for a very very very rought estimate just to see the rough motions of mechanism
        this.drivetrain=drivetrain;
        double momentOfInertia = 0.00001;
    
        pivotSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, 
        1./IntakeConstants.pivotGearReduction),  DCMotor.getNEO(1) , 0.004);
    }

    double simulatedPositionDegrees = 90;
    double pivotVelDegPerSec = 0;

    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double deltaT = 0.02;
        pivotSim.update(deltaT);

        inputs.pivotAppliedVolts = pivotSim.getInputVoltage();

        inputs.pivotAngleDegrees += pivotVelDegPerSec * deltaT;
        inputs.pivotAngleRadians += Units.degreesToRadians(pivotVelDegPerSec * deltaT);
        // System.out.println(inputs.pivotAngleDegrees);
        inputs.pivotVelocityDegreesPerSecond = pivotVelDegPerSec;

        Translation2d intakeTranslation2d = drivetrain.getPoseMeters().getTranslation().plus(new Translation2d(0.4,0));

        Pose3d intakePoseOnRobotSIM = new Pose3d(new Translation3d(intakeTranslation2d.getX(),intakeTranslation2d.getY(),0.3), 
            new Rotation3d(0,inputs.pivotAngleRadians,drivetrain.getPoseMeters().getRotation().getRadians()));

        Logger.recordOutput("intakeInputs/intakePoseOnRobotSIM", intakePoseOnRobotSIM);
    }

    @Override
    public void setPivotVolts(double volts) {

        if ((volts > -0.3) && (volts < 0.3)) {
            pivotVelDegPerSec = 0;
            pivotSim.setInputVoltage(0);
            return;
        }
        pivotSim.setInputVoltage(volts);

        pivotVelDegPerSec = volts * 30.0; // volts to deg
    }

    @Override
    public void setPivotPosition(double positionDegrees) {
        simulatedPositionDegrees = positionDegrees;
    }

    @Override
    public void setTopGripperVolts(double volts) {

    }

    @Override
    public void setBottomGripperVolts(double volts) {

    }
}
