package frc.robot.drivetrain;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.crevolib.util.SDSConstants;
import frc.robot.drivetrain.swerve.SwerveModuleConfig;

public class DrivetrainConfig {
    public class DriveConstants {
        //TODO: figure out PigeonID and change this
        public static final int pigeonID = 13; 

        //SDS Constants Class Made to Easily Switch Module Gear Ratios
        public static final SDSConstants chosenModule = SDSConstants.MK4i.Falcon500(SDSConstants.MK4i.driveRatios.L3); 

        /* Drivetrain Constants */
        //TODO: Change Robot Frame's Width and Length Depending on Frame Size Design makes
        public static final double trackWidth = Units.inchesToMeters(22.75);
        public static final double wheelBase = Units.inchesToMeters(22.75);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * Only works for square or rectangular frame (do not worry about this) */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /*Slow Mode Modifiers */
        public static final double kSlowModeTranslationModifier = 0.25;
        public static final double kSlowModeRotationModifier = 0.5;

        /*Intake Mode Modifiers */
        public static final double kIntakeModeTranslationModifier = 0.75;
        public static final double kIntakeModeRotationModifier = 0.6;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting - Angle/Drive Motors */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

      
        /* A small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        //TODO: TUNE THIS BASED ON TESTING
        public static final double driveKP = 0.08;; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.1;
        public static final double driveKF = 0.001;

        /* Drive Motor Characterization Values From SYSID */
        //TODO: This must be tuned to specific robot using SYSID, But Knight Vision Constants work good :)
        // public static final double driveKS = 0.48665; 
        // public static final double driveKV = 2.4132;
        // public static final double driveKA = 0.06921;

        public static final double driveKS = 0.48665; 
        public static final double driveKV = 2.4132;
        public static final double driveKA = 0.06921;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(18.0);
        /** Radians per Second */
        public static final double maxAngularVelocity = Math.PI * 4.12 * 0.5;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /*Swerve Profiling Values*/
        public static final double MAX_SPEED = (Units.feetToMeters(18.0)); //Max from SDS Limit Speed
        public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12 * 0.5;


        //TODO: TUNE THESE TO THE IDs FOR EACH MOTOR AND CANCODER and FIGURE OUT THE OFFSET
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(82.05);
            public static final SwerveModuleConfig config = 
                new SwerveModuleConfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.18);
            public static final SwerveModuleConfig config = 
                new SwerveModuleConfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-45.18);
            public static final SwerveModuleConfig config = 
                new SwerveModuleConfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-12.48);
            public static final SwerveModuleConfig config = 
                new SwerveModuleConfig(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public class AutonConstants {
        
    }
}
