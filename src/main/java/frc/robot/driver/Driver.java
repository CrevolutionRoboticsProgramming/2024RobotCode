package frc.robot.driver;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.crevolib.util.Gamepad;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;

public class Driver extends Gamepad {
    private static class Settings {
        static final int port = 0;
        static final String name = "driver";
    }

    private static Driver mInstance;

    private Driver() {
        super(Settings.name, Settings.port);

        
    }

    public static Driver getInstance() {
        if (mInstance == null) {
            mInstance = new Driver();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        leftBumperOnly().onTrue(new InstantCommand(() -> Drivetrain.getInstance().zeroHeading()));
        
        leftTriggerOnly().whileTrue(DrivetrainCommands.driveSlowMode(
            this::getDriveTranslation, 
            this::getDriveRotation));

        // right triger run intake
        // right bumper spit for amp form intake
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {

    }

    public Translation2d getDriveTranslation() {
        final var xComponent = -controller.getLeftX();
        final var yComponent = -controller.getLeftY();
        return new Translation2d((Math.abs(yComponent) < 0.1) ? 0 : yComponent, (Math.abs(xComponent) < 0.1) ? 0 : xComponent);
    }

    public double getDriveRotation() {
        return -controller.getRightX();
    }
}
