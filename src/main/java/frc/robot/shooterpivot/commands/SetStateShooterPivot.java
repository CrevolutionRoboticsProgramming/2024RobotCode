package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;

public class SetStateShooterPivot extends Command {
    public enum State {
        kHandoff(Rotation2d.fromDegrees(0)),
        kPrime(Rotation2d.fromDegrees(20)),
        kAmp(Rotation2d.fromDegrees(40));

        final Rotation2d target;
        State(Rotation2d target) {
            this.target = target;
        }
    }

    private final Rotation2d kAllowedError = Rotation2d.fromDegrees(2);

    private final ShooterPivot pivot;
    private final State targetState;

    private TrapezoidProfile profile;
    private Long startTs;

    SetStateShooterPivot(State target) {
        pivot = ShooterPivot.getInstance();
        targetState = target;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        if (isWithinAllowedError(targetState.target)) {
            return;
        }
        profile = getProfile();
        startTs = null;
    }

    @Override
    public void execute() {
        if (profile == null) {
            return;
        }
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }
        pivot.setAngularVelocity(Rotation2d.fromDegrees(profile.calculate(getElapsedTime()).velocity));
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
        if (isWithinAllowedError(ShooterPivot.Settings.kMaxAngle) && requestedState.velocity >= 0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setAngularVelocity(Rotation2d.fromDegrees(0));
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    private TrapezoidProfile getProfile() {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ShooterPivot.Settings.kMaxAngularVelocity.getDegrees(),
                ShooterPivot.Settings.kMaxAngularAcceleration.getDegrees()
            ),
            new TrapezoidProfile.State(targetState.target.getDegrees(), 0.0),
            new TrapezoidProfile.State(pivot.getAngle().getDegrees(), pivot.getAngularVelocity().getDegrees())
        );
    }

    private boolean isWithinAllowedError(Rotation2d target) {
        return Math.abs(pivot.getAngle().minus(target).getDegrees()) < kAllowedError.getDegrees();
    }
}
