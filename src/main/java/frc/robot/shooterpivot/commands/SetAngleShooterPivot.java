package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;

public class SetAngleShooterPivot extends Command {
    private final static boolean loggingEnabled = true;

    private static class Settings {
        // Unit: (deg / sec) / deg
        static final double kP = 5.0;
        static final double kI = 0.0;
        static final double kD = 0.0;
    }

    public enum Preset {
        kHandoff(Rotation2d.fromDegrees(0)),
        kHandoffClear(Rotation2d.fromDegrees(10)),
        kShooterNear(Rotation2d.fromDegrees(5)),
        kShooterFar(Rotation2d.fromDegrees(28.5)),
        kTrap(Rotation2d.fromDegrees(85)),
        kAmp(Rotation2d.fromDegrees(85));

        private final Rotation2d target;

        Preset(Rotation2d target) {
            this.target = target;
        }

        double getDegrees() {
            return target.getDegrees();
        }
    }

    private enum State {
        kProfile, kHold, kDone
    }

    private enum ProfileDirection {
        kPositive, kNegative;
    }

    private final ShooterPivot pivot;
    private State state;
    private final Preset targetState;

    private long startTs;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State initialProfileState;
    private final Rotation2d kProfileThreshold = Rotation2d.fromDegrees(4);

    private final PIDController pidController;
    private final Rotation2d kAllowedError = Rotation2d.fromDegrees(1);
    private final boolean indefinite;

    SetAngleShooterPivot(Preset target, boolean indefinite) {
        pivot = ShooterPivot.getInstance();
        targetState = target;
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            ShooterPivot.Settings.kMaxAngularVelocity.getDegrees(),
            ShooterPivot.Settings.kMaxAngularAcceleration.getDegrees()
        ));
        pidController = new PIDController(Settings.kP, Settings.kI, Settings.kD);
        this.indefinite = false;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        log("init (angle = %.2f)".formatted(targetState.getDegrees()));
        if (isWithinProfileThreshold()) {
            changeState(State.kHold, "within error threshold");
            return;
        }
        startTs = System.currentTimeMillis();
        initialProfileState = new TrapezoidProfile.State(pivot.getAngle().getDegrees(), pivot.getAngularVelocity().getDegrees());
        changeState(State.kProfile);
    }

    @Override
    public void execute() {
        switch (state) {
            case kProfile:
                final var elapsedTime = getElapsedTime();
                final var request = profile.calculate(
                    elapsedTime,
                    initialProfileState,
                    new TrapezoidProfile.State(targetState.target.getDegrees(), 0.0)
                );
                pivot.setAngularVelocity(Rotation2d.fromDegrees(request.velocity));

                if (profile.isFinished(elapsedTime)) {
                    changeState(State.kHold, "completed profile");
                    break;
                }

                final var currentAngle = pivot.getAngle().getDegrees();
                final var profileDir = (initialProfileState.position > targetState.target.getDegrees()) ? ProfileDirection.kPositive : ProfileDirection.kNegative;
                if (profileDir == ProfileDirection.kPositive && currentAngle >= ShooterPivot.Settings.kMaxAngle.getDegrees()) {
                    changeState(State.kHold, "hit positive stop");
                    break;
                } else if (currentAngle <= 0) {
                    changeState(State.kHold, "hit negative stop");
                    break;
                }
                break;
            case kHold:
                pivot.setAngularVelocity(Rotation2d.fromDegrees(pidController.calculate(pivot.getAngle().getDegrees(), targetState.getDegrees())));
                if (!indefinite && isWithinAllowedError()) {
                    changeState(State.kDone, "within allowed error");
                }
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.kDone;
    }

    @Override
    public void end(boolean interrupted) {
        log("end (interrupted = %b)".formatted(interrupted));
        pivot.setAngularVelocity(Rotation2d.fromDegrees(0));
    }

    public boolean isWithinAllowedError() {
        return Math.abs(pivot.getAngle().getDegrees() - targetState.getDegrees()) < kAllowedError.getDegrees();
    }

    private double getElapsedTime() {
        return (System.currentTimeMillis() - startTs) / 1000.0;
    }

    private boolean isWithinProfileThreshold() {
        return Math.abs(pivot.getAngle().getDegrees() - targetState.getDegrees()) < kProfileThreshold.getDegrees();
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
            System.out.printf("[%s] %s: %s%n", pivot.getClass().getSimpleName(), this.getClass().getSimpleName(), message);
        }
    }
}
