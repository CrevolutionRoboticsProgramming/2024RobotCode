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
import edu.wpi.first.math.util.Units;
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

        //Aiming Constants
        //TODO: Tune camera height + angle and target height constants
        public static final double cameraHeight_M = Units.inchesToMeters(0); //camera height
        final double targetHeight_M = Units.feetToMeters(0); //target height
        
        //Camera angle
        public static final double cameraPitch_R = Units.degreesToRadians(0);

        //desired distance from robot to target
        public static final double targetRange_M = Units.feetToMeters(0);

        //TODO: tune PID constants
        public static final double linearP = 0; //P term constant
        public static final double linearD = 0; //D term constant

        public static final double angularP = 0; //angle P term constant
        public static final double angularD = 0; //angle D term constant
    }
    public class DriverCamConfig {
        public static final String driverCamName = "Cam1";
    }
    
}
