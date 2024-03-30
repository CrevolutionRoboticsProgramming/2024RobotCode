package frc.robot.drivetrain.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.vision.Vision;

public class DriveAndHoldAngle extends Command {
    private static class Settings {
        static final double kP = 5.0;
        static final double kI = 0.0;
        static final double kD = 1.5;
        
        static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(360.0);
        static final Rotation2d kAllowedError = Rotation2d.fromDegrees(1.0);
    }

    private final Drivetrain drivetrain;
    private Rotation2d deltaTheta;
    private Supplier<Translation2d> translationSupplier;

    private final PIDController pidController;

    private Rotation2d targetAngle;

    public DriveAndHoldAngle(Supplier<Translation2d> translationSupplier) {
        drivetrain = Drivetrain.getInstance();
        this.deltaTheta = null;
        this.translationSupplier = translationSupplier;

        pidController = new PIDController(Settings.kP, Settings.kI, Settings.kD);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
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
        deltaTheta = endAngle.minus(startingAngle);

        targetAngle = Rotation2d.fromDegrees(drivetrain.getGyroYaw().getDegrees() + deltaTheta.getDegrees());

        final var currentAngle = drivetrain.getGyroYaw();
        final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
            pidController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()),
            -Settings.kMaxAngularVelocity.getDegrees(),
            Settings.kMaxAngularVelocity.getDegrees()
        ));
        drivetrain.drive(translationSupplier.get(), requestedAngularVelocity.getRadians(), false, false);
        // DrivetrainCommands.drive(translationSupplier, 
        //     () -> {return requestedAngularVelocity.getRadians();},
        //     1.0,
        //     1.0,
        //     true,
        //     new Translation2d(0, 0)
        // );

        SmartDashboard.putNumber("[TurnAnglePID] Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("[TurnAnglePID] Current Angle", currentAngle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        // if (deltaTheta.getDegrees() <= Settings.kAllowedError.getDegrees()) {
        //     return true;
        // }else {
        //     return false;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerve();
    }
}