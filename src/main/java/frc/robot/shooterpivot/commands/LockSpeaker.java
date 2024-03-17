package frc.robot.shooterpivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterflywheel.ShooterInterpolation;
import frc.robot.shooterpivot.ShooterPivot;

public class LockSpeaker extends Command {
    private static class Settings {
        static final double kP = 5.0;
        static final double kI = 0.0;
        static final double kD = 0.0;
        static final Rotation2d kMaxAngularVelocity = ShooterPivot.Settings.kMaxAngularVelocity.times(0.5);
    }

    private ShooterPivot shooterPivot;
    private Supplier<Rotation2d> angleSupplier;

    private PIDController velocityPIDController;


    LockSpeaker() {
        this(LockSpeaker::getTargetAngleDefault);
    }

    LockSpeaker(Supplier<Rotation2d> angleSupplier) {
        this.shooterPivot = ShooterPivot.getInstance();
        this.velocityPIDController = new PIDController(Settings.kP, Settings.kI, Settings.kD);
        this.angleSupplier = angleSupplier;
        
        addRequirements(shooterPivot);
    }

    @Override
    public void execute() {
        final var currentAngle = shooterPivot.getAngle();
        final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
            velocityPIDController.calculate(currentAngle.getDegrees(), angleSupplier.get().getDegrees()),
            -Settings.kMaxAngularVelocity.getDegrees(),
            Settings.kMaxAngularVelocity.getDegrees()
        ));
        shooterPivot.setAngularVelocity(requestedAngularVelocity);
    }

    public boolean isFinished() {
        return false;
    }

    private static Rotation2d getTargetAngleDefault() {
        // TODO: implement add interpolation calculations here
        return Rotation2d.fromDegrees(
                ShooterInterpolation.getInstance().getInterpolatedAngle(
                    ShooterPivot.getInstance().getDistanceFromSpeaker()
                )
            );
    }
}