package frc.robot.Intake.Commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Intake.IntakePivot;

public class SetPivotState extends Command{
    private final IntakePivot pivot;
    private final IntakeConfig.PivotState targetPivotState;

    private TrapezoidProfile profile;
    private final ArmFeedforward ffController;
    private final PIDController pidController;

    private Long startTs;

    public SetPivotState(IntakePivot pivot, IntakeConfig.PivotState targetPivotState) {
        this.pivot = pivot;
        this.targetPivotState = targetPivotState;

        ffController = new ArmFeedforward(IntakeConfig.kS, IntakeConfig.kG, IntakeConfig.kV, IntakeConfig.kA);
        pidController = new PIDController(IntakeConfig.kVelP, IntakeConfig.kVelI, IntakeConfig.kVelD);

        startTs = null;

        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return "SetPivotState[" + targetPivotState.name() + "]";
    }

    @Override
    public void initialize() {
        System.out.println(getName() + ": init");
        profile = generateProfile();

        startTs = null;
    }

    @Override
    public void execute() {
        final var duration = getElapsedTime();
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(duration);

        final var ffOutput = ffController.calculate(pivot.getAngleRads(), targetState.velocity);
        final var pidOutput = pidController.calculate(pivot.getVelocityRps(), targetState.velocity);

        pivot.setOutput((ffOutput + pidOutput) / 12.0);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(getName() + ": end");

        final IntakeConfig.PivotState state;
        if (pivot.getAngleRads() <= 0) {
            state = IntakeConfig.PivotState.kDeployed;
        } else if (interrupted) {
            System.out.println("pivot set state interrupted");
            state = IntakeConfig.PivotState.kUnspecified;
        } else {
            state = targetPivotState;
        }

        pivot.setState(state);
        pivot.stop();
    }

    @Override
    public boolean isFinished() {
        final var time = getElapsedTime();

        if (profile != null && time > 0.25 && profile.calculate(time).velocity < 0 && pivot.getAngleRads() < 0) {

            return true;
        }

        // 1) Profile is still running and reached limit switch
        return profile != null && profile.isFinished(time);
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile generateProfile() {
        final var startPos = pivot.getAngleRads();
        final var endPos = targetPivotState.target;

        final var distance = Math.min(Math.abs(endPos - startPos), (2 * Math.PI) - Math.abs(endPos - startPos));

        System.out.println("Start: " + startPos + ", End: " + endPos + ", Dist: " + distance);

        final TrapezoidProfile.State startState, goalState;
        if (startPos > 5.8 || startPos < endPos) {
            goalState = new TrapezoidProfile.State(distance, 0.0);
            startState = new TrapezoidProfile.State(0.0, pivot.getVelocityRps());
        } else {
            goalState = new TrapezoidProfile.State(0.0, 0.0);
            startState = new TrapezoidProfile.State(distance, pivot.getVelocityRps());
        }

        return new TrapezoidProfile(new TrapezoidProfile.Constraints(IntakeConfig.kMaxAngularVelocity, IntakeConfig.kMaxAngularAcceleration));
    }
}
