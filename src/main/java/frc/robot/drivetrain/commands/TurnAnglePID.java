package frc.robot.drivetrain.commands;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.vision.Vision;

public class TurnAnglePID extends Command {
    private static class Settings {
        static final double kP = 8.0;
        static final double kI = 0.0;
        static final double kD = 1.0;
        
        static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(360.0);

        static final Rotation2d kAllowedError = Rotation2d.fromDegrees(1.0);
    }

    private final Drivetrain drivetrain;
    private Rotation2d deltaTheta;

    private final PIDController pidController;

    private Rotation2d targetAngle;

    public TurnAnglePID() {
        drivetrain = Drivetrain.getInstance();
        this.deltaTheta = null;

        pidController = new PIDController(Settings.kP, Settings.kI, Settings.kD);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d goalPose;
        final var mPoseEstimator = Vision.PoseEstimator.getInstance();
        final var robotPose = mPoseEstimator.getCurrentPose();
        final var currentAlliance = DriverStation.getAlliance();
        // if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        // }
        // else {
        //     goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        // }

        goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        final var startingAngle = robotPose.getRotation();
        final var endAngle = goalPose.getTranslation().minus(robotPose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180.0));
        deltaTheta = endAngle.minus(startingAngle);

        targetAngle = Rotation2d.fromDegrees(drivetrain.getGyroYaw().getDegrees() + deltaTheta.getDegrees());
    }

    @Override
    public void execute() {
        final var currentAngle = drivetrain.getGyroYaw();
        final var requestedAngularVelocity = Rotation2d.fromDegrees(MathUtil.clamp(
            pidController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees()),
            -Settings.kMaxAngularVelocity.getDegrees(),
            Settings.kMaxAngularVelocity.getDegrees()
        ));
        drivetrain.drive(new Translation2d(0, 0), requestedAngularVelocity.getRadians(), false, false);

        SmartDashboard.putNumber("[TurnAnglePID] Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("[TurnAnglePID] Current Angle", currentAngle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        final var error = Math.abs(drivetrain.getGyroYaw().getDegrees() - targetAngle.getDegrees());
        return error < Settings.kAllowedError.getDegrees();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerve();
    }
}