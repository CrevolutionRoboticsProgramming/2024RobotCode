package frc.robot.Elevator;

import edu.wpi.first.math.util.Units;
import frc.CrevoLib.math.Conversions;

public class ElevatorConfig {
    //needs reviewing
    public enum ElevatorState {
        kZero(0),
        kTrap(0.9),
        kHigh(1),
        kUnspecified(0);
        
        public final double target;

        ElevatorState(double target) {
            this.target = target;
        }
    }

    //motor IDs have to be changed
    public static final int kElevatorSparkID1 = 51;
    public static final int kElevatorSparkID2 = 52;

    public static final boolean kElevatorMotorInverted = true;
    public static final boolean kElevatorEncoderInverted = false;
    
    //needs reviewing
    public static final double kSprocketDiameter = Units.feetToMeters(1.273 / 12.0);
    
    public static final int kLowerLimitSwitchPort = 53;
    public static final int kUpperLimitSwitchPort = 54;

    // Pivot PID constants
    public static final double kVelP = 0.1;
    public static final double kVelI = 0.0;
    public static final double kVelD = 0.0;

    // Pivot Feedforward Constants (reca.lc)
    public static final double kG = 0.23;
    public static final double kS = 0.0;
    public static final double kV = 2.92;
    public static final double kA = 0.01;

     // Max velocity and acceleration
     public static final double kMaxVelocity = 0.5;
     public static final double kMaxAcceleration = 0.6;

    // Sensor Parameters
    public static final boolean kPivotEncoderInverted = true;
    public static final double kPivotZeroOffset = 0.95;

    // Tensioning constants
    public static final int kDefaultContinuousCurrentLimit = 35;
    public static final int kDefaultPeakCurrentLimit = 60;

     public static final double kTensionOutput = -0.02;
     public static final double kTensionFindOutput = -0.05;
 
     public static final int kTensionContinuousCurrentLimit = 1;
     public static final int kTensionPeakCurrentLimit = 1;

   
}
