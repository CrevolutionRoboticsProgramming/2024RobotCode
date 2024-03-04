package frc.robot.intakepivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivot;

import java.util.function.Supplier;

public class SetVelocityIntakePivot extends Command {
    private final IntakePivot pivot;
    private final Supplier<Rotation2d> velocitySupplier;
    private final boolean openLoop;

    private final Rotation2d kScaleThreshold = Rotation2d.fromDegrees(20);

    SetVelocityIntakePivot(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        pivot = IntakePivot.getInstance();
        this.velocitySupplier = velocitySupplier;
        this.openLoop = openLoop;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        // Unset the requested angle so we don't snap back to set point
        pivot.setRequestedAngle(null);
    }

    @Override
    public void execute() {
        final var adjusted = scaleVelocityRequest(velocitySupplier.get());
        pivot.setVelocity(adjusted, openLoop);
    }

    public Rotation2d scaleVelocityRequest(Rotation2d velocity) {
        final var currentAngle = pivot.getAngle();
        if (velocity.getDegrees() > 0) {
            if (currentAngle.getDegrees() >= IntakePivot.Settings.kMaxAngle.getDegrees()) {
                return Rotation2d.fromDegrees(0);
            } else if (IntakePivot.Settings.kMaxAngle.getDegrees() - currentAngle.getDegrees() < kScaleThreshold.getDegrees()) {
                final var scaleFactor = (IntakePivot.Settings.kMaxAngle.getDegrees() - currentAngle.getDegrees()) / kScaleThreshold.getDegrees();
                return velocity.times(scaleFactor);
            }
        }
        if (velocity.getDegrees() < 0 && currentAngle.getDegrees() < kScaleThreshold.getDegrees()) {
            final var scaleFactor = currentAngle.getDegrees() / kScaleThreshold.getDegrees();
            return velocity.times(scaleFactor);
        }
        return velocity;
    }
}

