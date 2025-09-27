package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSim implements IntakeIO{
    
    private FlywheelSim pivotSim;

    public IntakeIOSim() {
        // fake values for sim for a very very very rought estimate just to see the rough motions of mechanism
        double momentOfInertia = 0.00001;
        FlywheelSim pivotSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, 
        1./IntakeConstants.pivotGearReduction),  DCMotor.getNEO(1) , 0.004);
    }

    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void setPivotVolts(double volts) {

    }

    @Override
    public void setPivotPosition(double positionDegrees) {

    }

    @Override
    public void setTopGripperVolts(double volts) {

    }

    @Override
    public void setBottomGripperVolts(double volts) {

    }
}
