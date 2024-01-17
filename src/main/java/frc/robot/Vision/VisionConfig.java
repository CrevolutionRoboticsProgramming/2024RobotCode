package frc.robot.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Drivetrain.DrivetrainConfig;
import frc.robot.Drivetrain.DrivetrainConfig.DriveConstants;

public class VisionConfig{
    public class ShooterCamsConfig {
        public static final String shooterCam1Name = "Cam1";
        public static final String shooterCam2Name = "Cam2";

        //Robot to Cam 1 constants
        public static final Transform3d cam1ToRobot = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
        public static final Transform3d robotToCam1 = cam1ToRobot.inverse();

        //Robot to Cam 2 constants
        public static final Transform3d cam2ToRobot = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d());
        public static final Transform3d robotToCam2 = cam2ToRobot.inverse();

        //Aiming Constants
        //TODO: Set max velocity and acceleration to TrapezoidProfile.Constraints
        public static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(0, 0);
        public static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(0, 0);
        public static final TrapezoidProfile.Constraints omegaConstraints =   new TrapezoidProfile.Constraints(0, 0);

        
        public static final double linearP = DriveConstants.driveKP; //P term constant
        public static final double linearD = DriveConstants.driveKD; //D term constant

        public static final double angularP = DriveConstants.angleKP; //angle P term constant
        public static final double angularD = DriveConstants.angleKD; //angle D term constant
    }
    public class DriverCamConfig {
        public static final String driverCamName = "Cam3";
    }
    
}
