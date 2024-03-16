package frc.robot.intakepivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivot;

public class SetStateIntakePivot extends Command {
    public enum State {
        kDeployed(Rotation2d.fromDegrees(0)),
        kSpit(Rotation2d.fromDegrees(90)),
        kStowed(Rotation2d.fromDegrees(182));

        final Rotation2d target;

        State(Rotation2d target) {
            this.target = target;
        }
    }

    private enum ProfileDirection {
        kPositive,
        kNegative
    }

    private enum CommandState {
        kInit,
        kProfile,
        kSeek,
        kDone
    }

    private static final boolean loggingEnabled = true;

    private final Rotation2d kAllowedError = Rotation2d.fromDegrees(2);

    private final IntakePivot pivot;
    private final State targetState;

    private Long startTs;
    private TrapezoidProfile profile;
    private ProfileDirection profileDirection;

    private TrapezoidProfile.State initProfileState, targetProfileState;

    private CommandState commandState;

    public SetStateIntakePivot(State targetState) {
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
        System.out.println("[IntakePivot] set state (state = " + targetState.name() + ")");
        profileDirection = getProfileDirection();
        profile = generateProfile();
        targetProfileState = new TrapezoidProfile.State(targetState.target.getDegrees(), 0.0);
        initProfileState = getCurrentState();
        startTs = null;
        commandState = CommandState.kInit;
        
    }

    @Override
    public void execute() {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        switch (commandState) {
            case kProfile:
                final var setState = profile.calculate(getElapsedTime(), initProfileState, targetProfileState);
                pivot.setVelocity(Rotation2d.fromDegrees(setState.velocity));
                pivot.setRequestedAngle(Rotation2d.fromDegrees(setState.position));
                break;
            case kSeek:
                final var dir = getProfileDirection();
                final var seekVel = (dir == ProfileDirection.kPositive) ? Rotation2d.fromDegrees(30) : Rotation2d.fromDegrees(-30);
                pivot.setVelocity(seekVel);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[IntakePivot] end set state (interrupted = " + interrupted + ")");
        pivot.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        switch (commandState) {
            case kInit:
                if (isWithinAllowedError(targetState.target)) {
                    changeState(CommandState.kDone, "within allowed error");
                    break;
                }
                if (profile != null) {
                    changeState(CommandState.kProfile);
                    break;
                }
                break;
            case kProfile:
                if (profile.isFinished(getElapsedTime())) {
                    if (isWithinAllowedError(targetState.target) || targetState == State.kSpit) {
                        changeState(CommandState.kDone);
                        break;
                    }
                    changeState(CommandState.kSeek);
                    break;
                }
                // Exit early if the profile is going to drive through a hard / soft angle limit
                final var requestedState = profile.calculate(getElapsedTime(), initProfileState, targetProfileState);
                if (isWithinAllowedError(Rotation2d.fromDegrees(0)) && requestedState.velocity < 0 && profileDirection == ProfileDirection.kNegative) {
                    changeState(CommandState.kDone, "hit deployed hard stop");
                    break;
                }
                if (isWithinAllowedError(IntakePivot.Settings.kMaxAngle) && requestedState.velocity > 0 && profileDirection == ProfileDirection.kPositive) {
                    changeState(CommandState.kDone, "hit stowed hard stop");
                    break;
                }
                break;
            case kSeek:
                if (isWithinAllowedError(targetState.target)) {
                    changeState(CommandState.kDone);
                    break;
                }
                break;
        }
        return commandState == CommandState.kDone;
    }

    private void changeState(CommandState newState) {
        changeState(newState, "");
    }

    private void changeState(CommandState newState, String reason) {
        if (loggingEnabled) {
            final var sb = new StringBuilder();
            sb.append("[IntakePivot] state change (");
            sb.append(commandState.name());
            sb.append(" -> ");
            sb.append(newState.name());
            if (!reason.isEmpty()) {
                sb.append(", reason: ");
                sb.append(reason);
            }
            sb.append(")");
            System.out.println(sb);
        }
        commandState = newState;
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile generateProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntakePivot.Settings.kMaxAngularVelocity.getDegrees(),
                IntakePivot.Settings.kMaxAngularAcceleration.getDegrees()
            )
        );
    }

    private TrapezoidProfile.State getCurrentState() {
        return new TrapezoidProfile.State(pivot.getAngle().getDegrees(), pivot.getAngularVelocity().getDegrees());
    }

    private ProfileDirection getProfileDirection() {
        if (targetState.target.getDegrees() > pivot.getAngle().getDegrees()) {
            return ProfileDirection.kPositive;
        }
        return ProfileDirection.kNegative;
    }

    private boolean isWithinAllowedError(Rotation2d target) {
        return Math.abs(pivot.getAngle().minus(target).getDegrees()) < kAllowedError.getDegrees();
    }
}
