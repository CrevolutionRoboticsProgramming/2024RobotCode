package frc.robot.driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.Robot;
import frc.robot.commands.RobotCommands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStateIntakePivot;
import frc.robot.intakeroller.commands.IntakeRollerCommands;

public class Driver extends Gamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";

        static final double kTranslationExpVal = 2.0;
        static final double kRotationExpVal = 1.0;
        static final double kDeadzone = 0.1;
    }

    private static Driver mInstance;
    private final ExpCurve translationStickCurve, rotationStickCurve;

    private Driver() {
        super(Settings.name, Settings.port);

        translationStickCurve = new ExpCurve(Settings.kTranslationExpVal, 0, 1, Settings.kDeadzone);
        rotationStickCurve = new ExpCurve(Settings.kRotationExpVal, 0, 1, Settings.kDeadzone);
    }

    public static Driver getInstance() {
        if (mInstance == null) {
            mInstance = new Driver();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        // Drivetrain Commands
        controller.triangle().onTrue(new InstantCommand(() -> Drivetrain.getInstance().zeroHeading()));
        controller.L2().whileTrue(DrivetrainCommands.driveSlowMode(this::getDriveTranslation, this::getDriveRotation));

        // Intake Commands
        controller.R2().whileTrue(IntakeRollerCommands.setOutput(() -> -1));
        controller.R2().onTrue(IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed));
        controller.R2().onFalse(IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed));

        controller.R1().onTrue(RobotCommands.spitNote());

        controller.circle().onTrue(RobotCommands.handOffNote());

        controller.povDown().whileTrue(DrivetrainCommands.turnToAngle(Rotation2d.fromDegrees(28.51)));
    }

    @Override
    public void setupDisabledButtons() {}

    @Override
    public void setupTestButtons() {}

    public Translation2d getDriveTranslation() {
        final var xComponent = translationStickCurve.calculate(-controller.getLeftX());
        final var yComponent = translationStickCurve.calculate(-controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public double getDriveRotation() {
        return rotationStickCurve.calculate(-controller.getRightX());
    }
}
