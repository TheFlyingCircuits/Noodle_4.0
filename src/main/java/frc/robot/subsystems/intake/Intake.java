package frc.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;
    
    ArmFeedforward pivotFeedForward; // in radians

    ProfiledPIDController pivotProfiledPID; // in radians

    TrapezoidProfile pivotProfile;

    PIDController pivotPID;

    private double desiredPivotAngleDegrees = 98.5;
    private double desiredTopGripperVolts = 0.0;
    private double desiredBottomGripperVolts = 0.0;

    Timer timer;

    double lastLoopTime = 0;

    double lastLoopVelocityRadPerSec = 0;

    boolean hasACoral = false;

    double pChange = 0.5;

    boolean shouldntAutoGoToPos = false;

    boolean currentlyIntaking = false;


    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
        timer = new Timer();

        pivotFeedForward = new ArmFeedforward(IntakeConstants.kSPivotVolts, IntakeConstants.kGPivotVolts,IntakeConstants.kVPivotVoltsSecondsPerRadian, IntakeConstants.kAPivotVoltsSecondsSquaredPerRadian);
        pivotProfiledPID = new ProfiledPIDController(IntakeConstants.kPPivotVoltsPerRadian, 0, IntakeConstants.kDPivotVoltsPerRadianPerSecond, 
            new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocityRadianPerSecond, IntakeConstants.maxPivotAccelerationRadianPerSecondSquared));
        pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocityRadianPerSecond, IntakeConstants.maxPivotAccelerationRadianPerSecondSquared));
        pivotPID = new PIDController(IntakeConstants.kPPivotVoltsPerRadian, 0, IntakeConstants.kDPivotVoltsPerRadianPerSecond);
        pivotPID.setTolerance(0.1);
        timer.start();

        // pivotProfiledPID.setTolerance(0); // 0.3 deg tolerance
        // pivotProfiledPID.reset(inputs.pivotAngleRadians, 0.0);
        // double fillerCalcToFixMaybe = pivotProfiledPID.calculate(Units.degreesToRadians(-19.0),
        //     new TrapezoidProfile.State(Units.degreesToRadians(-19.0), 0));
    }

    @Override
    public void periodic() {

        inputs.desiredPivotAngleDeg = desiredPivotAngleDegrees;
        inputs.desiredTopGripperVolts = desiredTopGripperVolts;
        inputs.desiredBottomGripperVolts = -desiredBottomGripperVolts;
        inputs.hasACoral = hasACoral;

        io.updateInputs(inputs);

        Logger.processInputs("intakeInputs", inputs);

        if(!shouldntAutoGoToPos){
            goToDesiredPivotAngle(currentlyIntaking);
        }
        setGripperVolts(inputs.desiredTopGripperVolts, inputs.desiredBottomGripperVolts);
    }

    public void setPivotVolts(double volts) {
        // System.out.println(volts);
        //+ 0.155* Math.cos(inputs.pivotAngleRadians)
        io.setPivotVolts(volts);
    }

    public void setGripperVolts(double topGripperVolts, double bottomGripperVolts) {
        io.setTopGripperVolts(topGripperVolts);
        io.setBottomGripperVolts(bottomGripperVolts);
    }

    public void setPivotTargetAngleDegrees(double targetDegrees) {
        if((targetDegrees > IntakeConstants.maxPivotAngleDegrees) || (targetDegrees < IntakeConstants.minPivotAngleDegrees)) {
            System.out.println("desired angle out of range");
            return;
        }

        desiredPivotAngleDegrees = targetDegrees;
    }

    public void setAvePivotAmpsForSim(double amps) {
        inputs.aveTopGripperAmps = amps;
        inputs.aveBottomGripperAmps = amps;
    }

    public boolean isIntakeDown() {
        return Math.abs(IntakeConstants.intakePivotSetpointDeg - inputs.pivotAngleDegrees) < 10.0;
        // return true;
    }

    public boolean doesHaveACoral() {
        return inputs.hasACoral;
    }

    public void goToDesiredPivotAngle(boolean isIntaking) {
        // System.out.println(-(Units.degreesToRadians(desiredPivotAngleDegrees) - inputs.pivotAngleRadians));
        // pivotProfiledPID.setP(pChange);
        // if (shouldSetNoVolts) {
        //     io.setPivotVolts(0);
        //     return;
        // }
        //  double profilePIDOutputVolts = pivotProfiledPID.calculate(Units.degreesToRadians(inputs.pivotAngleDegrees),
            // new TrapezoidProfile.State(Units.degreesToRadians(inputs.desiredPivotAngleDeg), 0));
        // double profilePIDOutputVolts = pivotProfiledPID.calculate(inputs.pivotAngleRadians, (Units.degreesToRadians(inputs.desiredPivotAngleDeg)));

        // we use .calculate for that and .setpoint for feed forward because .setpoint is where it should be on the profile but .calculate includes pid and is in volts
        // System.out.println(pivotProfiledPID.getSetpoint().velocity);

        TrapezoidProfile.State profileSetpointVelRadPerSec = pivotProfile.calculate(0.02, new TrapezoidProfile.State(Units.degreesToRadians(inputs.pivotAngleDegrees), Units.degreesToRadians(inputs.pivotVelocityDegreesPerSecond)),
             new TrapezoidProfile.State(Units.degreesToRadians(inputs.desiredPivotAngleDeg), 0));

        double pIDOutputVolts = pivotPID.calculate(Units.degreesToRadians(inputs.pivotAngleDegrees), Units.degreesToRadians(inputs.desiredPivotAngleDeg));
        double setpointVel = profileSetpointVelRadPerSec.velocity;
        if(Math.abs(inputs.desiredPivotAngleDeg - inputs.pivotAngleDegrees) < 2) {
            setpointVel = 0;
        } 
        // double profileSetpointVelRadPerSec = pivotProfile.getSetpoint().velocity;
        // get acceleration by delta velcocity from thisloop-last loop divided by delta time from thislooptime-lastlooptime
        double feedForwardVolts = pivotFeedForward.calculate(inputs.pivotAngleRadians, setpointVel);
        // double feedForwardVolts = pivotFeedForward.calculate(inputs.pivotAngleRadians, profileSetpointVelRadPerSec);
        Logger.recordOutput("goToDesiredPos/ pidOutputVolts", pIDOutputVolts);
        Logger.recordOutput("goToDesiredPos/ feedForwardVolts", feedForwardVolts);
        Logger.recordOutput("goToDesiredPos/ setpointVelocity", Units.radiansToDegrees(profileSetpointVelRadPerSec.velocity));



        lastLoopTime = timer.get();
        lastLoopVelocityRadPerSec = profileSetpointVelRadPerSec.velocity;

        // System.out.println(inputs.pivotAngleRadians + " rad");

        // System.out.println(feedForwardVolts);
        // System.out.println(profilePIDOutputVolts);
        double outputVoltsCombined = pIDOutputVolts + feedForwardVolts;
        if(isIntaking) {
            outputVoltsCombined = MathUtil.clamp(outputVoltsCombined, -12,-1);
        }
        setPivotVolts(outputVoltsCombined);
    }

    public void score(Supplier<Boolean> facingReef, boolean readyToScore, boolean manualOveride) {
        // this function needs to be called in a loop like execute or periodic
        currentlyIntaking = false;
        // System.out.println("ijegiojweogijwoigjwiogej");
        double scoringPivotDegrees = (facingReef.get()) ? IntakeConstants.frontScorePivotSetpointDeg : IntakeConstants.backScorePivotSetpointDeg; 
        double topGripperScoringVolts = (facingReef.get()) ? IntakeConstants.frontScoreTopGripperVolts : IntakeConstants.backScoreTopGripperVolts;
        double bottomGripperScoringVolts = (facingReef.get()) ? IntakeConstants.frontScoreBottomGripperVolts : IntakeConstants.backScoreBottomGripperVolts;
        
        desiredPivotAngleDegrees = scoringPivotDegrees;
        if((Math.abs(inputs.pivotAngleDegrees - scoringPivotDegrees) < 2.5 && readyToScore) || manualOveride) { // checks if the pivot is within 1 deg of target
            desiredTopGripperVolts = topGripperScoringVolts;
            desiredBottomGripperVolts = bottomGripperScoringVolts;
            hasACoral = false; // we assume that we wont have a coral after we start to score
        } else {
            desiredTopGripperVolts = IntakeConstants.holdCoralGripperVolts;
            desiredBottomGripperVolts = IntakeConstants.holdCoralGripperVolts;
        }

    }

    public void homeIntake(double positionDeg){
        // sets the Actuall position for homing
        shouldntAutoGoToPos = true;
        io.setPivotVolts(-3);
        io.setPivotPosition(positionDeg);
    }

    public void defaultFunction(boolean isInIntakeFunction) {
        // TODO: find real amp values for when we have and don't have a coral
        // if(hasACoral && (inputs.aveBottomGripperAmps < 10.0 && inputs.aveTopGripperAmps < 10.0)) { // if we have low amps while trying to grip we prob dont have coral
        //     hasACoral = false; // this is for if we drop the coral while doing defualt command the code adjusts by itself
        // }
        currentlyIntaking = false;
        shouldntAutoGoToPos = false;
        io.setCoastMode(false);
        if (hasACoral) {
            if(isInIntakeFunction) {
                desiredPivotAngleDegrees = IntakeConstants.hasCoralPivotSetpointDeg;
                desiredBottomGripperVolts = 0;
                desiredTopGripperVolts = -8;
                return;
            }
            desiredPivotAngleDegrees = IntakeConstants.hasCoralPivotSetpointDeg;
            desiredBottomGripperVolts = IntakeConstants.holdCoralGripperVolts;
            desiredTopGripperVolts = Math.abs(desiredPivotAngleDegrees - inputs.pivotAngleDegrees) < 5?
                IntakeConstants.intakingTopGripperVolts + 8: IntakeConstants.holdCoralGripperVolts;
            return;
        } else {
            desiredPivotAngleDegrees = IntakeConstants.noCoralPivotSetpointDeg;
            desiredTopGripperVolts = 0.0;
            desiredBottomGripperVolts = 0.0;
        }
        // System.out.println(desiredPivotAngleDegrees);

    }

    public void intakeCoral() {
        currentlyIntaking = true;
        io.setCoastMode(true);
        if (inputs.aveBottomGripperAmps > 40.0 || inputs.aveTopGripperAmps > 40.0) {
            hasACoral = true;
            defaultFunction(true);
            return;
        }

            desiredPivotAngleDegrees = IntakeConstants.intakePivotSetpointDeg;
            desiredTopGripperVolts = IntakeConstants.intakingTopGripperVolts;
            desiredBottomGripperVolts = IntakeConstants.intakingBottomGripperVolts;
            hasACoral = false;
             if (Math.abs(desiredPivotAngleDegrees - inputs.pivotAngleDegrees) < 3) {
                 shouldntAutoGoToPos = true;
            }
    }

    public void ejectCoral() {
        desiredTopGripperVolts = IntakeConstants.frontScoreTopGripperVolts;
        desiredBottomGripperVolts = IntakeConstants.frontScoreBottomGripperVolts;
    }

    public void intakeClimbPos(){
        desiredPivotAngleDegrees = 172;
    }


    public Command setTargetAngleDegCommand(double degrees) {
        return this.run(() -> setPivotTargetAngleDegrees(degrees));
    }

    public Command defaultCommand() {
        return this.run(() -> defaultFunction(false));
    }

    public Command intakeCommand() {
        return this.run(() -> intakeCoral());
    }

    public Command intakeScoreCommand(Supplier<Boolean> isFacingReef, boolean readyToScore, boolean manualOveride) {
        return this.run(() -> score(isFacingReef, readyToScore, manualOveride));
    }

    public Command ejectCoralCommand() {
        return this.run(() -> ejectCoral());
    }

    public Command intakeClimbCommand() {
        return this.run(() -> intakeClimbPos());
    }

    public Command homeIntakeCommand(double posDeg) {
        return this.run(() -> homeIntake(posDeg));
    }
}
