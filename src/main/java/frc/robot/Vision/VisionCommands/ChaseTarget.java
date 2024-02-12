package frc.robot.Vision.VisionCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Autos.AutonConfig;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Vision.VisionConfig.ShooterCamsConfig;

public class ChaseTarget extends Command{
    
    private PhotonCamera shooterCam1 = ShooterCamsConfig.shooterCam1;
    private Drivetrain drivetrain;
    private Supplier<Pose2d> poseEst;
    private Transform3d targetGoal =
     new Transform3d(new Translation3d(0.0, 0.0, Units.inchesToMeters(53)),
     new Rotation3d(0.0, 0.0, 0.0));
    
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
            omegaController.setTolerance(Units.degreesToRadians(3));
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

        if (result.hasTargets() && 
        checkID(ShooterCamsConfig.targetList, result.getBestTarget().getFiducialId())) {
            //get camera position
            var camPose = robotPose.transformBy(ShooterCamsConfig.robotToCam1);
            
            //get target position
            var camToTarget = result.getBestTarget().getBestCameraToTarget();
            var targetPose = camPose.transformBy(camToTarget);

            //set goals
            var goalPose = targetPose.transformBy(targetGoal).toPose2d();

            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());

            //print vision vars
            System.out.println("Goal Pose: " + goalPose.getRotation().getDegrees());
            System.out.println("Cam Pose: " + camPose);
            System.out.println("Target Pose: " + targetPose);
            System.out.println("Robot Pose: " + robotPose2d.getRotation().getDegrees());
            System.out.println("Cam to Target " + camToTarget);

            //drive to target
            // var xSpeed = xController.calculate(robotPose.getX());
            // if (xController.atGoal()){
            //     xSpeed = 0;
            // }

            // var ySpeed = yController.calculate(robotPose.getY());
            // if (yController.atGoal()){
            //     ySpeed = 0;
            // }

            var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()){
                drivetrain.stopSwerve();
            }
            if(goalPose.getRotation().getRadians() < 0) {
                omegaSpeed = -omegaSpeed;
            }

            drivetrain.drive(new Translation2d(0, 0), omegaSpeed, true, true);

            // Shuffleboard.getTab(ShooterCamsConfig.shuffleboardTabName).add("Omega Goal", omegaSpeed);

            
                
        } else {
            // if we have no targets, don't move
            drivetrain.stopSwerve();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopSwerve();
    }
}
