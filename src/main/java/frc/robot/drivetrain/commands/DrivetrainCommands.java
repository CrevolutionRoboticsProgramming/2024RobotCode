package frc.robot.drivetrain.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.vision.Vision;

public class DrivetrainCommands {
    private static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotationSupplier, double translationModifier,
                                 double rotationModifier, boolean isFieldOriented, Translation2d rotationOffset) {
        return new TeleopDrive(
            () -> translationSupplier.get().times(DriveConstants.MAX_SPEED).times(translationModifier),
            () -> Rotation2d.fromRadians(rotationSupplier.getAsDouble()).times(DriveConstants.MAX_ANGULAR_VELOCITY).times(rotationModifier),
            isFieldOriented,
            rotationOffset
        );
    }

    public static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation) {
        return drive(translationSupplier, rotation, 1.0, 1.0, true, new Translation2d(0, 0));
    }

    public static Command driveSlowMode(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation) {
        return drive(
            translationSupplier,
            rotation,
            DriveConstants.kSlowModeTranslationModifier,
            DriveConstants.kSlowModeRotationModifier,
            true,
            new Translation2d(0, 0)
        );
    }

    public static Command turnToAngle(Rotation2d target) {
        return new TurnAngle(target);
    }

    public static Command autoLineUp() {
        Pose2d goalPose;
        var mPoseEstimator = Vision.PoseEstimator.getInstance();
        var robotPose = mPoseEstimator.getCurrentPose();
        var currentAlliance = DriverStation.getAlliance();
        if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        }
        else {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        }

        final var startingAngle = robotPose.getRotation();
        final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
        final var deltaTheta = endAngle.minus(startingAngle).times(-1);

        /*Other way to calculate */
        // startingAngle = robotPose.getRotation().getRadians();
        // double adjacent = Math.abs(robotPose.getX() - goalPose.getX());
        // double opposite = Math.abs(robotPose.getY() - goalPose.getY());
        // endAngle = Math.atan2(adjacent, opposite);
        // var deltaTheta = (endAngle - startingAngle);
        // deltaTheta *= Math.signum((robotPose.getX()) - (goalPose.getX()));

        return new TurnAngle(deltaTheta);
    }
}
