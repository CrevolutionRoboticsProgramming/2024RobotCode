package frc.robot.Vision.VisionCommands;

import java.util.ArrayList;
import java.util.logging.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModuleState.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autos.AutonConfig;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.DrivetrainConfig;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;
import frc.robot.Elevator.ElevatorConfig;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionConfig;
import frc.robot.Vision.Vision.PoseEstimator;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class VisionLineUp extends Command {
    private final Drivetrain mDrivetrain;
    private final PoseEstimator mPoseEstimator;
    // private final PIDController pidController;
    // private final SimpleMotorFeedforward ffController;
    private TrapezoidProfile profile;
    private Pose2d robotPose;
    private Pose2d goalPose;
    private double startingAngle, endAngle, distance;
    private Long startTs;

    // Robot Radius (diagonal) in meters
    private final double radius = 0.97 / 2.0; 
    

    public VisionLineUp(Drivetrain mDrivetrain, PoseEstimator mPoseEstimator) {
        this.mDrivetrain = mDrivetrain;
        this.mPoseEstimator = mPoseEstimator;
        profile = null;
        startTs = null;

        addRequirements(mDrivetrain, mPoseEstimator);
    }

    @Override
    public void initialize() {
        robotPose = mPoseEstimator.getCurrentPose();
        var currentAlliance = DriverStation.getAlliance();
        if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        }
        else {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        }
        
        startingAngle = robotPose.getRotation().getDegrees();
        double adjacent = Math.abs((robotPose.getX()) - (goalPose.getX()));
        double opposite = Math.abs((robotPose.getY()) - (goalPose.getY()));
        endAngle = Math.atan2(adjacent, opposite);

        var deltaTheta = Units.degreesToRadians(endAngle - startingAngle);
        distance = radius * deltaTheta;
        profile = generateProfile(distance);
        
        System.out.println(deltaTheta);
    }   

    @Override
    public void execute() {
        if (startTs == null) {
            startTs = System.currentTimeMillis();
        }

        final var targetState = profile.calculate(getElapsedTime());
        mDrivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(targetState.velocity, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(targetState.velocity, new Rotation2d(Units.degreesToRadians(225))),
            new SwerveModuleState(targetState.velocity, new Rotation2d(Units.degreesToRadians(45))),
            new SwerveModuleState(targetState.velocity, new Rotation2d(Units.degreesToRadians(135)))
        });
    }

    private double getElapsedTime() {
        return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
    }

    @Override
    public boolean isFinished() {
        final var time = getElapsedTime();
        return profile != null && profile.isFinished((double) time);
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.stopSwerve();
    }

    private TrapezoidProfile generateProfile(double targetDistance) {
        return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(DriveConstants.maxSpeed, 0.55),
            new TrapezoidProfile.State(targetDistance, 0),
            new TrapezoidProfile.State(0, 0)
        );
    }
    
}