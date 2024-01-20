package frc.robot.Vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Vision.VisionConfig.*;

public class ShooterCams extends SubsystemBase {
    //Define our PhotonCameras
    public static PhotonCamera shooterCam1 = new PhotonCamera(ShooterCamsConfig.shooterCam1Name);
    public static PhotonCamera shooterCam2 = new PhotonCamera(ShooterCamsConfig.shooterCam2Name);
    
    
    // Declare AprilTagFieldLayout outside the try block
    public static AprilTagFieldLayout aprilTagField;

    public ShooterCams() {
        // Initialize AprilTagFieldLayout in the constructor
        try {
            aprilTagField = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            // Handle the exception appropriately, e.g., log or throw a runtime exception
            throw new RuntimeException("Failed to load AprilTagFieldLayout", e);
        }
        }
}
