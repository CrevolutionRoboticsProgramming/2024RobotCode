package frc.robot.drivetrain.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.vision.Vision;

public class DrivetrainCommands {
    public static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotationSupplier, double translationModifier,
                                 double rotationModifier, boolean isFieldOriented, Translation2d rotationOffset, boolean modeS, boolean modeA) {
        return new TeleopDrive(
            () -> translationSupplier.get().times(DriveConstants.MAX_SPEED).times(translationModifier),
            () -> Rotation2d.fromRadians(rotationSupplier.getAsDouble()).times(DriveConstants.MAX_ANGULAR_VELOCITY).times(rotationModifier),
            isFieldOriented,
            rotationOffset,
            modeS,
            modeA
        );
    }

    public static Command stopSwerve(Supplier<Translation2d> translationSupplier) {
        return drive(
            translationSupplier, 
            () -> 0.0,
            1.0,
            1.0,
            true,
            new Translation2d(0, 0),
            false,
            false
        );
    }

    public static Command driveAndLockTarget(Supplier<Translation2d> translationSupplier) {
        // final PIDController pidController = new PIDController(4.825, 0.0, 1.5);
        // final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(360.0);
        // final Rotation2d kAllowedError = Rotation2d.fromDegrees(2.0);
        // final Drivetrain drivetrain = Drivetrain.getInstance();
        // System.out.println("Is it gonna run???: true");
        // if (getRelativeAngleToSpeaker().getDegrees() > kAllowedError.getDegrees()) {
        //     System.out.println("Is it gonna work?: true");
        //     return drive(
        //     translationSupplier, 
        //     () -> {
        //         final var deltaTheta = getRelativeAngleToSpeaker();
        //         final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
        //             pidController.calculate(0.0, deltaTheta.getDegrees()),
        //             -kMaxAngularVelocity.getDegrees(),
        //             kMaxAngularVelocity.getDegrees()
        //         ));
        //         return requestedAngularVelocity.getRadians();
        //     },
        //     1.0,
        //     1.0,
        //     true,
        //     new Translation2d(0, 0)
        // );
        // } else {
        //     System.out.println("Is it gonna work?: False");
        //     return stopSwerve(translationSupplier);
        // }
        return new DriveAndHoldAngle(translationSupplier);
    }

    private static Rotation2d getRelativeAngleToSpeaker() {
        Pose2d goalPose = null;
        final var mPoseEstimator = Vision.PoseEstimator.getInstance();
        final var robotPose = mPoseEstimator.getCurrentPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {
                goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
            }
            if (ally.get() == Alliance.Red) {
                goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
            }
        }
        final var startingAngle = robotPose.getRotation();
        final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
        return endAngle.minus(startingAngle);
    }

    public static Command drive(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation) {
        return drive(translationSupplier, rotation, 1.0, 1.0, true, new Translation2d(0, 0), false, false);
    }

    public static Command shootAmpMode(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation, boolean modeS, boolean modeA) {
        return drive(translationSupplier, rotation, 1.0, 1.0, true, new Translation2d(0, 0), modeS, modeA);
    }

    public static Command shootSpeakerMode(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation, boolean modeS, boolean modeA) {
        return drive(translationSupplier, rotation, 1.0, 1.0, true, new Translation2d(0, 0), modeS, modeA);
    }

    public static Command driveSlowMode(Supplier<Translation2d> translationSupplier, DoubleSupplier rotation, boolean modeS, boolean modeA) {
        return drive(
            translationSupplier,
            rotation,
            DriveConstants.kSlowModeTranslationModifier,
            DriveConstants.kSlowModeRotationModifier,
            true,
            new Translation2d(0, 0),
            modeS,
            modeA
        );
    }

    public static Command turnToAngle() {
        return new TurnAnglePID();
    }

    public static Command autoLineUp() {
        // Pose2d goalPose;
        // var mPoseEstimator = Vision.PoseEstimator.getInstance();
        // var robotPose = mPoseEstimator.getCurrentPose();
        // var currentAlliance = DriverStation.getAlliance();
        // if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // }
        // else {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        // }

        // goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // var startingAngle = robotPose.getRotation();
        // if (startingAngle.getDegrees() < 0) {
        //     startingAngle = Rotation2d.fromDegrees(startingAngle.getDegrees() + 360);
        // }
        // var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle();
        // var deltaTheta = endAngle.minus(startingAngle).times(-1);

        // System.out.println("Starting Angle: " + startingAngle);
        // System.out.println("End Angle: " + startingAngle);
        // System.out.println("DeltaTheta Angle: " + startingAngle);

        // goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // var a = goalPose.getTranslation();

        // // vector calc
        // var b = robotPose.getTranslation();
        // double ab = (a.getX() * b.getX()) + (a.getY() * b.getY());
        // double magA = Math.sqrt( (Math.pow(a.getX(), 2) + Math.pow(a.getY(), 2)) );
        // double magB = Math.sqrt( (Math.pow(b.getX(), 2) + Math.pow(b.getY(), 2)) );

        // double deltaTheta = Math.acos(ab / (magA * magB));

        /*Other way to calculate */
        // var startingAngle = robotPose.getRotation().getRadians();
        // double adjacent = Math.abs(robotPose.getX() - goalPose.getX());
        // double opposite = Math.abs(robotPose.getY() - goalPose.getY());
        // var endAngle = Math.atan2(adjacent, opposite);
        // var deltaTheta = (endAngle - startingAngle);
        // deltaTheta *= Math.signum((robotPose.getX()) - (goalPose.getX()));
        
        return new TurnAnglePID();
    }

    public static Command holdPassPos() {
        return new TurnPassPID();
    }
}
