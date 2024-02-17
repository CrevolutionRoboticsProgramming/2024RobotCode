package frc.robot.Vision.VisionCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModuleState.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final PIDController pidController;
    private TrapezoidProfile profile;
    private Pose2d robotPose;
    private Pose2d goalPose;
    private State currentState, targetState;
    private double startingAngle, endAngle;
    private SwerveModuleState[] modState;
    

    public VisionLineUp(Drivetrain mDrivetrain, PoseEstimator mPoseEstimator) {
        this.mDrivetrain = mDrivetrain;
        this.mPoseEstimator = mPoseEstimator;

        modState = new SwerveModuleState[4];

        pidController = new PIDController(DriveConstants.driveKP, DriveConstants.driveKI, DriveConstants.driveKD);
        profile = null;

        addRequirements(mDrivetrain, mPoseEstimator);
    }

    @Override
    public void initialize() {
        profile = generateProfile();

        robotPose = mPoseEstimator.getCurrentPose();
        var currentAlliance = DriverStation.getAlliance();
        if(currentAlliance.equals(DriverStation.Alliance.Blue)) {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
        }
        else {
            goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
        }
    }   

    @Override
    public void execute() {
        final var duration = (double) System.currentTimeMillis() / 1000.0;

        startingAngle = robotPose.getRotation().getDegrees();
        double adjacent = Math.abs((robotPose.getX()) - (goalPose.getX()));
        double opposite = Math.abs((robotPose.getY()) - (goalPose.getY()));
        endAngle = Math.atan2(adjacent, opposite);

        final var targetState = profile.calculate(duration);

        final double pidOutput = pidController.calculate(Units.degreesToRadians(endAngle), targetState.velocity);

        modState[0] = new SwerveModuleState(0.0, new Rotation2d(pidOutput));
        modState[1] = new SwerveModuleState(0.0, new Rotation2d(pidOutput));
        modState[2] = new SwerveModuleState(0.0, new Rotation2d(pidOutput));
        modState[3] = new SwerveModuleState(0.0, new Rotation2d(pidOutput));

        mDrivetrain.setModuleStates(modState);

        // print angle valuess
        System.out.println("start angle" + startingAngle);
        System.out.println("end angle" + endAngle);

        
    }


    @Override
    public void end(boolean interrupted) {
        mDrivetrain.stopSwerve();
    }

    private TrapezoidProfile generateProfile() {
        return new TrapezoidProfile(new TrapezoidProfile.Constraints(DriveConstants.maxAngularVelocity, 720.0), 
        new TrapezoidProfile.State(Units.degreesToRadians(endAngle),0),
        new TrapezoidProfile.State(Units.degreesToRadians(startingAngle), DriveConstants.maxAngularVelocity));
    }
    
}