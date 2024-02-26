package frc.robot.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public class DrivetrainCommands {
    public static Command autoLineUp() {
        var robotPose = RobotContainer.poseEstimator.getCurrentPose();
        var currentAlliance = DriverStation.getAlliance();
        // if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // }
        // else {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        // }

        var goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        final var startingAngle =  robotPose.getRotation();
        final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
        final var deltaTheta = endAngle.minus(startingAngle).times(-1);

        /*Other way to calculate */
        // startingAngle = robotPose.getRotation().getRadians();
        // double adjacent = Math.abs(robotPose.getX() - goalPose.getX());
        // double opposite = Math.abs(robotPose.getY() - goalPose.getY());
        // endAngle = Math.atan2(adjacent, opposite);

        // var deltaTheta = (endAngle - startingAngle);
        // deltaTheta *= Math.signum((robotPose.getX()) - (goalPose.getX()));
        return new TurnToAngle(RobotContainer.mSwerveDrivetrain, deltaTheta);
    }

    public static Command drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        return drive(translationX, translationY, rotation, 1.0, 1.0, true, new Translation2d(0, 0));
    }

    public static Command driveSlowMode(DoubleSupplier translationX, DoubleSupplier translationY,
                                        DoubleSupplier rotation) {
        return drive(
                translationX,
                translationY,
                rotation,
                DriveConstants.kSlowModeTranslationModifier,
                DriveConstants.kSlowModeRotationModifier,
                true,
                new Translation2d(0, 0)
        );
    }

    public static Command driveIntakeMode(DoubleSupplier translationX, DoubleSupplier translationY,
                                          DoubleSupplier rotation) {
       return drive(
               translationX,
               translationY,
               rotation,
               DriveConstants.kIntakeModeTranslationModifier,
               DriveConstants.kIntakeModeRotationModifier,
               true,
               new Translation2d(
                       -(DriveConstants.trackWidth + 0.2) / 2.0,
                       0
               )
       );
    }

    // TODO: Legacy bug where Y and X inputs are inverted in the Translation2d
    private static Command drive(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            double translationModifier,
            double rotationModifier,
            
            boolean isFieldOriented,
            Translation2d rotationOffset) {
        return new TeleopDrive(RobotContainer.mSwerveDrivetrain, translationX, translationY, rotation, isFieldOriented);
    }
}
