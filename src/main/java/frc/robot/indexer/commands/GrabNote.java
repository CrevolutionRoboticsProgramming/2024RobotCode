package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.Indexer;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;

public class GrabNote extends Command {
    private final Indexer indexer;

    private enum State {
        kLoading, kReverse, kReLoad, kSlowDown, kStop, kDone
    }

    public enum Profile{
        kHighLoad(0.75),
        kLowLoad(0.15);

        double percent;
        Profile(double percentOut) {
            percent = percentOut;
        }
    }

    private State state;
    private static final boolean loggingEnabled = true;
    private final Profile profile;

    public GrabNote(Profile profile) {
        indexer = Indexer.getInstance();
        this.profile = profile;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        state = State.kLoading;
    }

    @Override
    public void execute() {
        indexer.setOutput(switch (state) {
            case kLoading -> 1.00;
            case kReverse -> -0.40;
            case kReLoad -> 1.0;
            case kSlowDown -> 0.5;
            case kStop -> 0.00;
            default -> 0.0;
        });
        transitionState();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        return state == State.kStop;
    }

    private void transitionState() {
        switch (state) {
            case kLoading:
                if (indexer.hasFinalNote()) {
                    changeState(State.kReverse, "detected note");
                }
                break;
            case kReverse:
                if (indexer.hasInitialNote()) {
                    changeState(State.kReLoad, "Detected note again");
                }
                break;
            case kReLoad:
                if (indexer.hasInitialNote()) {
                    changeState(State.kSlowDown, "detected note");
                }
                break;
            case kSlowDown:
                if (indexer.hasFinalNote()) {
                    changeState(State.kStop);
                }
        }
    }

    private void changeState(State newState) {
        changeState(newState, "");
    }

    private void changeState(State newState, String reason) {
        final var details = new StringBuilder();
        if (state != null) {
            details.append("old state = %s, ".formatted(state.name()));
        }
        details.append("state = %s, ".formatted(newState.name()));
        if (!reason.isEmpty()) {
            details.append("reason = %s".formatted(reason));
        }
        log("%s (%s)".formatted((state == null) ? "initial state" : "change state", details));
        state = newState;
    }

    private void log(String message) {
        if (loggingEnabled) {
            System.out.printf("[%s] %s: %s%n", indexer.getClass().getSimpleName(), this.getClass().getSimpleName(), message);
        }
    }
}