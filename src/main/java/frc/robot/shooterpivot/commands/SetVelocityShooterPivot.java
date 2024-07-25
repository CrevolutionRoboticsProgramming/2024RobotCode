package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;

import java.util.function.Supplier;

public class SetVelocityShooterPivot extends Command {
    private final ShooterPivot pivot;
    private final Supplier<Rotation2d> velocitySupplier;
    private final boolean openLoop;

    private final Rotation2d kScaleThreshold = Rotation2d.fromDegrees(10);

    SetVelocityShooterPivot(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        pivot = ShooterPivot.getInstance();
        this.velocitySupplier = velocitySupplier;
        this.openLoop = openLoop;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.setAngularVelocity(scaleVelocityRequest(velocitySupplier.get()), openLoop);
    }

    @Override
    public void initialize() {
        System.out.println("[shooterpivot] init set vel");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[shooterpivot] end set vel");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public Rotation2d scaleVelocityRequest(Rotation2d velocity) {
//        final var currentAngle = pivot.getAngle();
//        if (velocity.getDegrees() > 0) {
//            if (currentAngle.getDegrees() >= ShooterPivot.Settings.kMaxAngle.getDegrees()) {
//                return Rotation2d.fromDegrees(0);
//            } else if (ShooterPivot.Settings.kMaxAngle.minus(currentAngle).getDegrees() < kScaleThreshold.getDegrees()) {
//                final var scaleFactor = ShooterPivot.Settings.kMaxAngle.minus(currentAngle).getDegrees() / kScaleThreshold.getDegrees();
//                return velocity.times(scaleFactor);
//            }
//        }
//        if (velocity.getDegrees() < 0 && currentAngle.getDegrees() < kScaleThreshold.getDegrees()) {
//            final var scaleFactor = currentAngle.getDegrees() / kScaleThreshold.getDegrees();
//            return velocity.times(scaleFactor);
//        }
        return velocity;
    }
}
