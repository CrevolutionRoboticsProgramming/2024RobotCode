package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.Indexer;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;

public class LoadNote extends Command {
    private final Indexer indexer;

    private enum State {
        kLoading, kBackoff, kReverseBackoff, kDone
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
            case kLoading -> 0.60;
            // case kBackoff -> -profile.percent;
            // case kReverseBackoff -> 0.15;
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
        return state == State.kDone;
    }

    private void transitionState() {
        // switch (state) {
        //     case kLoading:
        //         if (indexer.hasNote()) {
        //             changeState(State.kBackoff, "detected note");
        //         }
        //         break;
        //     case kBackoff:
        //         if (!indexer.hasNote()) {
        //             changeState(State.kReverseBackoff, "no longer detected note");
        //         }
        //         break;
        //     case kReverseBackoff:
        //         if (indexer.hasNote()) {
        //             changeState(State.kDone, "detected note");
        //         }
        //         break;
        // }

        switch (state) {
            case kLoading:
                if (indexer.hasNote()) {
                    changeState(State.kDone, "detected note");
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