package frc.robot.intakepivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivot;

public class SetStatePivot extends Command {
    public enum State {
        kDeployed(Rotation2d.fromDegrees(0)),
        kSpit(Rotation2d.fromDegrees(90)),
        kStowed(Rotation2d.fromDegrees(180));

        final Rotation2d target;

        State(Rotation2d target) {
            this.target = target;
        }
    }

    private final Rotation2d kAllowedError = Rotation2d.fromDegrees(2);

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
        // Handle the early exit case, we're already within the allowed margin of error at the start of the command
        if (profile == null) {
            return isWithinAllowedError(targetState.target);
        }
        // Profile is complete, go ahead and exit
        if (profile.isFinished(getElapsedTime())) {
            return true;
        }
        // Exit early if the profile is going to drive through a hard / soft angle limit
        final var requestedState = profile.calculate(getElapsedTime());
        if (isWithinAllowedError(Rotation2d.fromDegrees(0)) && requestedState.velocity <= 0) {
            return true;
        }
        if (isWithinAllowedError(IntakePivot.Settings.kMaxAngle) && requestedState.velocity >= 0) {
            return true;
        }
        return false;
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile generateProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakePivot.Settings.kMaxAngularVelocity.getDegrees(),
                IntakePivot.Settings.kMaxAngularAcceleration.getDegrees()
            ),
            new TrapezoidProfile.State(targetState.target.getDegrees(), 0.0),
            new TrapezoidProfile.State(pivot.getAngle().getDegrees(), pivot.getAngularVelocity().getDegrees())
        );
    }

    private boolean isWithinAllowedError(Rotation2d target) {
        return Math.abs(pivot.getAngle().minus(target).getDegrees()) < kAllowedError.getDegrees();
    }
}
