package frc.robot.driver;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.crevolib.util.Gamepad;

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

    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {

    }

    public Translation2d getDriveTranslation() {
        return new Translation2d(-controller.getLeftX(), controller.getLeftY());
    }

    public double getDriveRotation() {
        return controller.getRightX();
    }
}
