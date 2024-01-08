package frc.robot.Vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionConfig{
    public class ShooterCamsConfig {
        public static final String shooterCam1Name = "Cam2";
        public static final String shooterCam2Name = "Cam3";

        //Robot to Cam 1 constants
        public static final Translation3d shooterCam1Translation = new Translation3d(0,0,0);
        public static final Rotation3d shooterCam1Rotation = new Rotation3d(0,0,0);

        //Robot to Cam 2 constants
        public static final Translation3d shooterCam2Translation = new Translation3d(0,0,0);
        public static final Rotation3d shooterCam2Rotation = new Rotation3d(0,0,0);
    }
    public class DriverCamConfig {
        public static final String driverCamName = "Cam1";
    }
    
}
