package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class VisionConfig{
    public class ShooterCamsConfig {
        public static final String shooterCam1Name = "Shooting-Cam";
        // public static final String shooterCam2Name = "Drive-Cam";

        public static PhotonCamera shooterCam1 = new PhotonCamera(shooterCam1Name);
        // public static PhotonCamera shooterCam2 = new PhotonCamera(shooterCam2Name);

        //Robot to Cam 1 constants
        public static final Transform3d cam1ToRobot = new Transform3d(
            new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(0), Units.inchesToMeters(19)), 
            new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(180))
        );
        public static final Transform3d robotToCam1 = cam1ToRobot.inverse();

        //Robot to Cam 2 constants
        public static final Transform3d cam2ToRobot = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
        public static final Transform3d robotToCam2 = cam2ToRobot.inverse();
        

        //PID Values for Vision
        public static final double kPTranslation = 0.2;
        public static final double kITranslation = 0;
        public static final double kDTranslation = 0;

        public static final double kPRotation = 0.1;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;
        //Aiming Constants
        //TODO: Set max velocity and acceleration to TrapezoidProfile.Constraints (currently set: default example code values)
        public static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints omegaConstraints =   new TrapezoidProfile.Constraints(8, 8);
        public static final TrapezoidProfile.Constraints xyConstraints = new TrapezoidProfile.Constraints(3, 2);

        public static final double fieldLength_m = Units.inchesToMeters(651.25);
        public static final double fieldWidth_m = Units.inchesToMeters(323.25);

        public static final Pose2d flippingPose = new Pose2d(
        new Translation2d(fieldLength_m, fieldWidth_m),
        new Rotation2d(Math.PI));

        //Target list
        public static Integer targetList[] = {6,7};
        //TODO: get exact poses for targets
        public static Pose3d target6Pose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)); 
        public static Pose3d target7Pose = new Pose3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        

        public static final String shuffleboardTabName = "Vision";
    }
    public class DriverCamConfig {
        public static final String driverCamName = "Cam3";
    }
    
}
