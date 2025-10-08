package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class ClimberIOSim implements ClimberIO{
    
    private Drivetrain drivetrain;

    public ClimberIOSim(Drivetrain drivetrain) {
        // fake values for sim for a very very very rought estimate just to see the rough motions of mechanism
        this.drivetrain=drivetrain;
    }

    double simulatedPositionDegrees = 90;
    double pivotVelDegPerSec = 0;

    
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double deltaT = 0.02;

        inputs.lifterAngleDeg += pivotVelDegPerSec * deltaT;


        double robotYawRad = drivetrain.getPoseMeters().getRotation().getRadians();

        Translation2d climberTranslation2d = drivetrain.getPoseMeters().getTranslation().plus(new Translation2d(Math.cos(robotYawRad)* 0.4,Math.sin(robotYawRad)*0.4));

        Pose3d climberPoseOnRobotSIM = new Pose3d(new Translation3d(climberTranslation2d.getX(),climberTranslation2d.getY(),0.3), 
            new Rotation3d(0,-Units.degreesToRadians(inputs.lifterAngleDeg),drivetrain.getPoseMeters().getRotation().getRadians()));

        Logger.recordOutput("climberInputs/climberPoseOnRobotSIM", climberPoseOnRobotSIM);
    }

    @Override
    public void setLifterNeoVolts(double volts) {

        pivotVelDegPerSec = volts * 20.0; // volts to deg
    }

    @Override
    public void setSuckerNeoVolts(double volts) {

    }


}
