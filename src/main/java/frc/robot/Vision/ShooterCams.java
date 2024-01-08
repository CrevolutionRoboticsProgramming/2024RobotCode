package frc.robot.Vision;

import java.io.IOException;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.VisionConfig.*;

public class ShooterCams extends SubsystemBase {
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
        PhotonPoseEstimator poseEst2 = new PhotonPoseEstimator(aprilTagField, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, CamConstants.camera2, CamConstants.robotToCam2);

        PhotonPoseEstimator poseEst3 = new PhotonPoseEstimator(aprilTagField, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, CamConstants.camera2, CamConstants.robotToCam2);

    }
}
