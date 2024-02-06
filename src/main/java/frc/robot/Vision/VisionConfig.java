package frc.robot.Vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Drivetrain.DrivetrainConfig;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;

public class VisionConfig{
    public class ShooterCamsConfig {
        public static final String shooterCam1Name = "Cam1";
        public static final String shooterCam2Name = "Cam2";

        public static PhotonCamera shooterCam1 = new PhotonCamera(shooterCam1Name);
        public static PhotonCamera shooterCam2 = new PhotonCamera(shooterCam2Name);

        //Robot to Cam 1 constants
        public static final Transform3d cam1ToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(11.25), Units.inchesToMeters(1.25), 0.0), new Rotation3d());
        public static final Transform3d robotToCam1 = cam1ToRobot.inverse();

        //Robot to Cam 2 constants
        public static final Transform3d cam2ToRobot = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
        public static final Transform3d robotToCam2 = cam2ToRobot.inverse();

        //Aiming Constants
        //TODO: Set max velocity and acceleration to TrapezoidProfile.Constraints (currently set: default example code values)
        public static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(3, 2);
        public static final TrapezoidProfile.Constraints omegaConstraints =   new TrapezoidProfile.Constraints(8, 8);

        
        public static final double linearP = DriveConstants.driveKP; //P term constant
        public static final double linearD = DriveConstants.driveKD; //D term constant

        public static final double angularP = DriveConstants.angleKP; //angle P term constant
        public static final double angularD = DriveConstants.angleKD; //angle D term constant

        public static final double fieldLength_m = Units.inchesToMeters(651.25);
        public static final double fieldWidth_m = Units.inchesToMeters(323.25);

        public static final Pose2d flippingPose = new Pose2d(
        new Translation2d(fieldLength_m, fieldWidth_m),
        new Rotation2d(Math.PI));

        public static final String shuffleboardTabName = "Vision";
    }
    public class DriverCamConfig {
        public static final String driverCamName = "Cam3";
    }
    
}
