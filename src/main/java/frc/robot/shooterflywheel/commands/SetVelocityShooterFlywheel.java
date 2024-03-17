package frc.robot.shooterflywheel.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterflywheel.ShooterFlywheel;

import java.util.function.Supplier;

public class SetVelocityShooterFlywheel extends Command {
    private final ShooterFlywheel flywheel;
    private final Supplier<Rotation2d> leftVelocitySupplier, rightVelocitySupplier;

    private final Rotation2d kAllowedError = Rotation2d.fromRotations(5); // 300 RPM

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
        final var leftVel = leftVelocitySupplier.get();
        final var rightVel = rightVelocitySupplier.get();
        flywheel.setRightFlywheelVelocity(leftVel);
        flywheel.setLeftFlywheelVelocity(rightVel);

        SmartDashboard.putBoolean("Shooter Ready (left)", Math.abs(leftVel.getRotations() - flywheel.getLeftFlywheelVelocity().getRotations()) < kAllowedError.getRotations());
        SmartDashboard.putBoolean("Shooter Ready (right)", Math.abs(rightVel.getRotations() - flywheel.getRightFlywheelVelocity().getRotations()) < kAllowedError.getRotations());
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
