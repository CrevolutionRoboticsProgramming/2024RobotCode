package frc.robot.drivetrain.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.vision.Vision;

public class DriveAndStopAim extends Command{
    public DriveAndStopAim() {

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        DriverXbox.getInstance().setDriveRotation(-DriverXbox.getInstance().controller.getRightX());

        DriverXbox.getInstance().autoAim = false;
    }

    @Override
    public boolean isFinished() {
        // final var error = Math.abs(drivetrain.getGyroYaw().getDegrees() - targetAngle.getDegrees());
        // return error < Settings.kAllowedError.getDegrees();
        return false;
    }
}
