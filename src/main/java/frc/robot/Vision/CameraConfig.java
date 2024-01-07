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

public class CameraConfig{
    public class CamConstants{
        // Camera 1
        //public static String camera1 = new PhotonCamera("cam1");
        

        // Camera 2
        public static PhotonCamera camera2 = new PhotonCamera("cam2");

        // Ints in parameters are camera location and rotation, change depending on where cam is
        public static Transform3d robotToCam2 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

        // Camera 3
        public static PhotonCamera camera3 = new PhotonCamera("cam3");

        // Ints in parameters are camera location and rotation, change depending on where cam is
        public static Transform3d robotToCam3 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    }
    
}
