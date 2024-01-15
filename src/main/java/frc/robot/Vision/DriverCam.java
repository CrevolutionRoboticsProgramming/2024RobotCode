package frc.robot.Vision;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.VisionConfig.*;


public class DriverCam extends SubsystemBase{

    //Sets a driver camera (we think, we actually have no idea what we did)
    public static PhotonCamera driveCamera = new PhotonCamera(DriverCamConfig.driverCamName);
    public static void setDriveCamera(PhotonCamera driveCamera) {
        DriverCam.driveCamera = driveCamera;
    }
    
}
