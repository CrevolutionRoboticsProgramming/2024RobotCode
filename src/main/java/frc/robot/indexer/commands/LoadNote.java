package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.Indexer;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;

public class LoadNote extends Command {
    private final Indexer indexer;

    private enum State {
        kLoading, kSlowDown, kStop, kDone
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

    public LoadNote(Profile profile) {
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
            case kSlowDown -> 0.50;
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
                if (indexer.hasInitialNote()) {
                    changeState(State.kSlowDown, "detected note");
                }
                break;
            case kSlowDown:
                if (indexer.hasFinalNote()) {
                    changeState(State.kStop, "Detected note again");
                }
                break;
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