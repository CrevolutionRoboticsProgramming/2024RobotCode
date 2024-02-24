package frc.robot.Drivetrain.DrivetrainCommands;

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

public class TurnToAngle extends Command {
    private final Drivetrain mDrivetrain;
    // private final PIDController pidController;
    // private final SimpleMotorFeedforward ffController;
    private TrapezoidProfile profile;
    Rotation2d deltaTheta;
    private double distance;
    private Long startTs;

    // Robot Radius (diagonal) in meters
    private final double radius = 0.97 / 2.0; 
    

    public TurnToAngle(Drivetrain mDrivetrain, Rotation2d rotate2d) {
        this.mDrivetrain = mDrivetrain;
        deltaTheta = rotate2d;

        addRequirements(mDrivetrain);
    }

    @Override
    public void initialize() {
        profile = null;
        startTs = null;

        distance = radius * deltaTheta.getRadians();
        profile = generateProfile(distance);
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