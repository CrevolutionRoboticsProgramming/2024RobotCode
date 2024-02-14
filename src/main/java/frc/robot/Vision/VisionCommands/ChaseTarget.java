package frc.robot.Vision.VisionCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.CrevoLib.util.SDSConstants.MK4i.driveRatios;
import frc.robot.Autos.AutonConfig;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.Vision.PoseEstimator;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class ChaseTarget extends Command{

    private Drivetrain swerve = new Drivetrain();
    private PhotonCamera shooterCam1 = ShooterCamsConfig.shooterCam1;
    private Drivetrain drivetrain;
    private Supplier<Pose2d> poseEst;
    private Transform3d targetGoal =
     new Transform3d(new Translation3d(0.0, 0.0, Units.inchesToMeters(53)),
     new Rotation3d(0.0, 0.0, Math.PI));
    private PhotonTrackedTarget lastTarget;
    
    //PID  Controllers
    static ProfiledPIDController xController = new ProfiledPIDController(ShooterCamsConfig.kPTranslation, ShooterCamsConfig.kITranslation, ShooterCamsConfig.kDTranslation, ShooterCamsConfig.xConstraints);
    static ProfiledPIDController yController = new ProfiledPIDController(ShooterCamsConfig.kPTranslation, ShooterCamsConfig.kITranslation, ShooterCamsConfig.kDTranslation, ShooterCamsConfig.yConstraints);
    static ProfiledPIDController omegaController = new ProfiledPIDController(ShooterCamsConfig.kPRotation, ShooterCamsConfig.kIRotation, ShooterCamsConfig.kDRotation, ShooterCamsConfig.omegaConstraints);

    public ChaseTarget(
        PhotonCamera shooterCam1,
        Supplier<Pose2d> poseEst,
        Drivetrain drivetrain){
            this.shooterCam1 = shooterCam1;
            this.drivetrain = drivetrain;
            this.poseEst = poseEst;
            //TODO: Set tolerances, example code tolerances currently set
            xController.setTolerance(0.2);
            yController.setTolerance(0.2);
            //omegaController.setTolerance(Units.degreesToRadians(3));
            omegaController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(drivetrain);
        }

    private static boolean checkID(Integer[] arr, int fiducialID){
        boolean acceptedID = Arrays.asList(arr).contains(fiducialID);
        return acceptedID;
    }

    @Override
    public void initialize() {
        //reset all poses
        var robotPose = poseEst.get();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute(){
        
        //get robot position
        var robotPose2d = poseEst.get();
        var robotPose = new Pose3d(
            robotPose2d.getX(), 
            robotPose2d.getY(),
            0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        
        //get last camera result
        var result = shooterCam1.getLatestResult();

        if (result.hasTargets()) {
            //verify target id
            var optionalTarget = result.getTargets().stream()
            .filter(t -> checkID(ShooterCamsConfig.targetList, t.getFiducialId()))
            .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
            .findFirst();
            if (optionalTarget.isPresent()){
                var target = optionalTarget.get();
                lastTarget = target;

                //get camera pose
                var cameraPose = robotPose.transformBy(ShooterCamsConfig.robotToCam1);
                System.out.println("Camera Pose: " + cameraPose);

                //get target pose
                var camToTarget = target.getBestCameraToTarget();
                var targetPose = cameraPose.transformBy(camToTarget);
                System.out.println("Target Pose: " + targetPose);

                //set goal pose
                var goalPose = targetPose.transformBy(targetGoal).toPose2d();
                System.out.println("Goal Pose: " + goalPose);
                
                omegaController.setGoal(goalPose.getRotation().getRadians());
            }
        }
        if (lastTarget == null){
            drivetrain.stopSwerve();
        } else {

            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()){
                end(true);
            }

            drivetrain.drive(new Translation2d(0, 0), omegaSpeed, true, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerve();
    }
    // public Rotation2d calculateRequiredHeading(){
    //     var pose = poseEstimator.getCurrentPose();
    //     return PhotonUtils.getYawToPose(pose, new Pose2d(0, 5.6, new Rotation2d(0,0)));
    // }
}
