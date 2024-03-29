package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public class TurnAngleProfile extends Command {
    private final Drivetrain mDrivetrain;
    private TrapezoidProfile profile;
    Rotation2d deltaTheta;
    private double distance;
    private Long startTs;

    // Robot Radius (diagonal) in meters
    private final double radius = 0.7112 / 2.0; 

    public TurnAngleProfile(Rotation2d deltaTheta, boolean continuous) {
        mDrivetrain = Drivetrain.getInstance();
        this.deltaTheta = deltaTheta;

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
        SmartDashboard.putNumber("[TurnAngle] requested vel", targetState.velocity);
        SmartDashboard.putNumber("[TurnAngle] requested pos", targetState.position);
    
        System.out.println("Distatnce: " + distance);
        System.out.println("Target Velocity: " + targetState.velocity);
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
            new TrapezoidProfile.Constraints(DriveConstants.maxSpeed, 0.1),
            new TrapezoidProfile.State(targetDistance, 0),
            new TrapezoidProfile.State(0, 0)
        );
    }
}