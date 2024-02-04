// package frc.robot.Shooter;

// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.XboxController;

// public class ShooterConfig {
// public enum PivotState {
        
//         kScoreLow(Units.degreesToRadians(60)),
//         // kHumanPlayer(Conversions.degreesToRadians(135)),
//         kShootSpeaker(Units.degreesToRadians(60));

        
//         /**
//          * Creates a PivotState with a target in radians
//          *
//          * @param target angle in radians
//          */
//         PivotState(double target) {
//             this.target = target;
//         }
//         public final double target;
//     }
//     public class ShooterProfile {
//         public ShooterProfile(int stallLimit, int freeLimit, double nominalSpeed, String name) {
//             kStallCurrentLimit = stallLimit;
//             kFreeCurrentLimit = freeLimit;
//             kNominalOutput = nominalSpeed;
//             kName = name;
//         }

//         @Override
//         public String toString() {
//             return kName;
//         }

//         final int kStallCurrentLimit;
//         final int kFreeCurrentLimit;
//         final double kNominalOutput;
//         private final String kName;
//     }

//  //motor IDs have to be changed
//     public static final int kLeftShooterSparkID = 26;
//     public static final int kRightShooterSparkID = 27;
//     public static final int kPivotSparkID = 28;
//     public static final int kIndexTalonID = 29;

//     public static final boolean kPivotMotorInverted = false;
//     public static final boolean kShooterMotorInverted = false;
//     public static final boolean kIndexerMotorInverted = false;

//     public static final CANSparkMax.IdleMode kPivotIdleMode = CANSparkMax.IdleMode.kBrake;
//     public static final CANSparkMax.IdleMode kRollerIdleMode = CANSparkMax.IdleMode.kCoast;

//     // Intake Profiles
//     // public static ShooterProfile kOuttake = new ShooterProfile(60, 60, -1, "OutTake");
//     // public static ShooterProfile kShoot = new ShooterProfile(60, 60, -.8, "ShootCube");
//     // public static ShooterProfile kHandoff = new ShooterProfile(40, 40, 0.5, "Handoff");
//     // public static ShooterProfile kDefaultProfile = new ShooterProfile(40, 40, 1, "Default");

//     public static final int shooterCurrentLimit = 35;
//     public static final int shooterCurrentThreshold = 60;
//     public static final double shooterCurrentThresholdTime = 0.1;
//     public static final boolean shooterEnableCurrentLimit = true;

//      public static final int kHOLimitSwitch = 53;

//      public static final double kMaxVelocity = 0.5;
//      public static final double kMaxAcceleration = 0.6;
 
//      /* Shooter PID Constants */
//      public static final double SHOOTER_P = 0.01;
//      public static final double SHOOTER_I = 0;
//      public static final double SHOOTER_D = 0;
//      public static final double SHOOTER_V = 0.01;
//      public static final double SHOOTER_A = 0;
 
//      /* Shooter Velocities - Rotations per Second */
//     //  public static final double SHOOTER_SHOOT;
 
// }
