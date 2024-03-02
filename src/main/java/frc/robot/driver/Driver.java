package frc.robot.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

public class Driver extends Gamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";

        static final double kTranslationExpVal = 2.0;
        static final double kRotationExpVal = 1.0;
        static final double kDeadzone = 0.1;
    }

    ExpCurve translationStickCurve;
    ExpCurve rotationStickCurve;
    ExpCurve testCurve;

    private static Driver mInstance;

    private Driver() {
        super(Settings.name, Settings.port);

        translationStickCurve = new ExpCurve(Settings.kTranslationExpVal, 0, 1, Settings.kDeadzone);
        rotationStickCurve = new ExpCurve(Settings.kRotationExpVal, 0, 1, Settings.kDeadzone);
        testCurve = new ExpCurve(1.0, 0, ShooterPivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
    }

    public static Driver getInstance() {
        if (mInstance == null) {
            mInstance = new Driver();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        controller.L1().onTrue(new InstantCommand(() -> Drivetrain.getInstance().zeroHeading()));

        controller.R2().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
            () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(getRightTriggerMagnitude())
        ));
        controller.L2().whileTrue(IndexerCommands.setOutput(this::getLeftTriggerMagnitude));

        controller.povRight().whileTrue(ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromDegrees(45), false));
        controller.povRight().whileTrue(ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromDegrees(-45), false));
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {
        // Shooter test
        rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.1).whileTrue(ShooterPivotCommands.setAngularVelocity(
            () -> Rotation2d.fromRadians(testCurve.calculate(controller.getRightX())), false
        ));
//        controller.R2().whileFalse(ShooterPivotCommands.setAngularVelocity(
//            () -> Rotation2d.fromRadians(0), false
//        ));
        controller.R2().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
            () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(getRightTriggerMagnitude())
        ));
        controller.L2().whileTrue(IndexerCommands.setOutput(() -> -getLeftTriggerMagnitude()));
    }

    public Translation2d getDriveTranslation() {
        final var xComponent = translationStickCurve.calculate(controller.getLeftX());
        final var yComponent = translationStickCurve.calculate(controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public double getDriveRotation() {
        return -rotationStickCurve.calculate(controller.getRightX());
    }
}
