package frc.robot.intakepivot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.IntakePivotConfig;
import frc.robot.intakeroller.IntakeConfig;

public class SetStatePivot extends Command{
    public enum State {
        kDeployed(Rotation2d.fromDegrees(0)),
        kSpit(Rotation2d.fromDegrees(20)),
        kStowed(Rotation2d.fromDegrees(90));

        final Rotation2d setpoint;
        State(Rotation2d setpoint) {
            this.setpoint = setpoint;
        }
    }

    private final IntakePivot pivot;
    private final State targetState;

    private Long startTs;
    private TrapezoidProfile profile;
    private Rotation2d initialState;


    public SetStatePivot(State targetState) {
        this.pivot = IntakePivot.getInstance();
        this.targetState = targetState;
        this.startTs = null;
        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return "SetPivotState(" + targetState.name() + ")";
    }

    @Override
    public void initialize() {
        profile = generateProfile();
        initialState = pivot.getAngle();
        startTs = null;
    }

    @Override
    public void execute() {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var setState = profile.calculate(getElapsedTime());
        pivot.setVelocity(Rotation2d.fromDegrees(setState.velocity));
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        final var time = getElapsedTime();

        // Safety stop, it's trying to drive into the robot
        if (profile != null && time > 0.25 && profile.calculate(time).velocity < 0 && pivot.getAngularVelocity().getDegrees() < 0) {
            return true;
        }

        // 1) Profile is still running and reached limit switch
        return profile != null && profile.isFinished(time);
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile generateProfile() {
        final var startPos = pivot.getAngle().getDegrees();
        final var endPos = targetState.setpoint.getDegrees();

        final var deltaTheta = Math.min(Math.abs(endPos - startPos), 360 - Math.abs(endPos - startPos));

        System.out.println("Start: " + startPos + ", End: " + endPos + ", Dist: " + deltaTheta);

        final TrapezoidProfile.State startState, goalState;
        if (startPos > 270 || startPos < endPos) {
            goalState = new TrapezoidProfile.State(deltaTheta, 0.0);
            startState = new TrapezoidProfile.State(0.0, pivot.getAngularVelocity().getDegrees());
        } else {
            goalState = new TrapezoidProfile.State(0.0, 0.0);
            startState = new TrapezoidProfile.State(deltaTheta, pivot.getAngularVelocity().getDegrees());
        }

        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(IntakeConfig.kMaxAngularVelocity, IntakeConfig.kMaxAngularAcceleration),
            goalState,
            startState
        );
    }
}
