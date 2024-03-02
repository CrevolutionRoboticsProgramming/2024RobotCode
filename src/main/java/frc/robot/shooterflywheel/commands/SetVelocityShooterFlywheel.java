package frc.robot.shooterflywheel.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterflywheel.ShooterFlywheel;

import java.util.function.Supplier;

public class SetVelocityShooterFlywheel extends Command {
    private final ShooterFlywheel flywheel;
    private final Supplier<Rotation2d> leftVelocitySupplier, rightVelocitySupplier;

    SetVelocityShooterFlywheel(Supplier<Rotation2d> leftVelocitySupplier, Supplier<Rotation2d> rightVelocitySupplier) {
        flywheel = ShooterFlywheel.getInstance();
        this.leftVelocitySupplier = leftVelocitySupplier;
        this.rightVelocitySupplier = rightVelocitySupplier;
    }

    SetVelocityShooterFlywheel(Supplier<Rotation2d> velocitySupplier) {
        this(velocitySupplier, velocitySupplier);
    }

    @Override
    public void execute() {
        flywheel.setRightFlywheelVelocity(leftVelocitySupplier.get());
        flywheel.setLeftFlywheelVelocity(rightVelocitySupplier.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        flywheel.setRightFlywheelVelocity(Rotation2d.fromDegrees(0));
        flywheel.setLeftFlywheelVelocity(Rotation2d.fromDegrees(0));
    }
}
