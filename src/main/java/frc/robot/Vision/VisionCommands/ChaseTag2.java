package frc.robot.Vision.VisionCommands;

import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.Vision.PoseEstimator;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class ChaseTag2 extends Command{
    
    private final PoseEstimator poseEstimator;
    private final Drivetrain drivetrain;
    private final PhotonCamera cam;
    private ProfiledPIDController PIDprofileX, PIDprofileY, PIDprofileR;
    private PhotonTrackedTarget target;
    private Pose2d startPose, currentPose;
    private Pose3d targetPose;
    private int target7 = 7;

    public ChaseTag2(PoseEstimator poseEstimator, Drivetrain drivetrain){
            this.poseEstimator = poseEstimator;
            this.drivetrain = drivetrain;

            cam = ShooterCamsConfig.shooterCam1;

            addRequirements(poseEstimator, drivetrain);
    }

    @Override
    public void initialize(){
        
        //make PID profiles for movement and rotation
        PIDprofileX = new ProfiledPIDController(ShooterCamsConfig.kPTranslation, 
        ShooterCamsConfig.kITranslation, 
        ShooterCamsConfig.kDTranslation, 
        ShooterCamsConfig.xyConstraints);

        PIDprofileY = new ProfiledPIDController(ShooterCamsConfig.kPTranslation, 
        ShooterCamsConfig.kITranslation, 
        ShooterCamsConfig.kDTranslation, 
        ShooterCamsConfig.xyConstraints);
        
        PIDprofileR = new ProfiledPIDController(ShooterCamsConfig.kPRotation,
        ShooterCamsConfig.kIRotation, 
        ShooterCamsConfig.kDRotation, 
        ShooterCamsConfig.omegaConstraints);
    }

    @Override
    public void execute(){

        //get the pose the robot's currently at
        startPose = poseEstimator.getCurrentPose();

        //get target data
        var optTarget = cam.getLatestResult().getBestTarget();

        //check target id
        //if ID = 7, target pose is target 6 pose and last target will be target 7
        if(optTarget.getFiducialId() == target7){
                System.out.println("Target ID" + optTarget.getFiducialId() + " found");
                target = optTarget;
                targetPose = ShooterCamsConfig.target7Pose;

        } else if(optTarget == null || optTarget.getFiducialId() != target7){
            //this will be rewritten so when it doesn't see a target,
            //it moves to where it can see the speaker tag
            end(true);
            System.out.println("No target found");
        }
        //calculate goal pose to align with middle speaker tag
        Pose3d startPose3d = new Pose3d(startPose);
        var targetGoalPose = targetPose.transformBy(target.getBestCameraToTarget());
        var goalPose = startPose3d.transformBy(new Transform3d(targetGoalPose.getTranslation(), targetGoalPose.getRotation()));

        var goalPose2d = goalPose.toPose2d();
        
        //drive to alignment
            //set goals
            PIDprofileX.setGoal(goalPose.getTranslation().getX());
            PIDprofileY.setGoal(goalPose.getTranslation().getY());
            //calculate needed outputs
            var xSpeed = PIDprofileX.calculate(goalPose.getX());
            var ySpeed = PIDprofileY.calculate(goalPose.getY());
        
        //rotate towards tag
            //set goal
            PIDprofileR.setGoal(goalPose.getRotation().getAngle());
            //calculate output
            var rSpeed = PIDprofileR.calculate(goalPose.getRotation().getAngle());
        
        drivetrain.drive(new Translation2d(xSpeed, ySpeed), rSpeed,  true, true);

        //end command
        currentPose = poseEstimator.getCurrentPose();
        if (currentPose == goalPose2d){
            end(true);
            drivetrain.stopSwerve();
        }
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.stopSwerve();
        isFinished();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
