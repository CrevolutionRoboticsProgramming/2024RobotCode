package frc.robot.Vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.VisionConfig.*;

public class ShooterCams extends SubsystemBase {
    //Define our PhotonCameras
    public static PhotonCamera shooterCam1 = new PhotonCamera(ShooterCamsConfig.shooterCam1Name);
    public static PhotonCamera shooterCam2 = new PhotonCamera(ShooterCamsConfig.shooterCam2Name);

    //Robot to Cam: 
    public static Transform3d robotToCam1 = new Transform3d(ShooterCamsConfig.shooterCam1Translation, ShooterCamsConfig.shooterCam1Rotation);
    public static Transform3d robotToCam2 = new Transform3d(ShooterCamsConfig.shooterCam2Translation, ShooterCamsConfig.shooterCam2Rotation);
    
    
    // Declare AprilTagFieldLayout outside the try block
    private final AprilTagFieldLayout aprilTagField;

    public ShooterCams() {
        // Initialize AprilTagFieldLayout in the constructor
        try {
            aprilTagField = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            // Handle the exception appropriately, e.g., log or throw a runtime exception
            throw new RuntimeException("Failed to load AprilTagFieldLayout", e);
        }

        // Pose estimator
        PhotonPoseEstimator poseEst2 = new PhotonPoseEstimator(aprilTagField, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, shooterCam1, robotToCam1);

        PhotonPoseEstimator poseEst3 = new PhotonPoseEstimator(aprilTagField, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, shooterCam2, robotToCam2);

    }
}