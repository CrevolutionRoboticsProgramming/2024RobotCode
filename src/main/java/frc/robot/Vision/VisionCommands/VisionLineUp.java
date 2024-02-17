package frc.robot.Vision.VisionCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autos.AutonConfig;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Elevator.ElevatorConfig;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionConfig;
import frc.robot.Vision.Vision.PoseEstimator;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class VisionLineUp extends Command {
    private final Drivetrain mDrivetrain;
    private final PoseEstimator mPoseEstimator;
    private Pose2d robotPose;
    private Pose2d goalPose;

    public VisionLineUp(Drivetrain mDrivetrain, PoseEstimator mPoseEstimator) {
        this.mDrivetrain = mDrivetrain;
        this.mPoseEstimator = mPoseEstimator;
        
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
    }   

    @Override
    public void execute() {
        double startingAngle = robotPose.getRotation().getDegrees();
        double adjacent = Math.abs((robotPose.getX()) - (goalPose.getX()));
        double opposite = Math.abs((robotPose.getY()) - (goalPose.getY()));
        double endAngle = Math.atan2(adjacent, opposite);
        mDrivetrain.drive(new Translation2d(0,0), Units.degreesToRadians(endAngle),true, true);
    }


    @Override
    public void end(boolean interrupted) {
        mDrivetrain.stopSwerve();
    }
    
}