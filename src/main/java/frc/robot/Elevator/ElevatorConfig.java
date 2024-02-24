package frc.robot.Elevator;

import com.revrobotics.CANSparkMax;

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
    
    // Sensor Parameters
    public static final boolean kPivotEncoderInverted = true;
    public static final double kPivotZeroOffset = 0.95;
    
    //needs reviewing
    public static final double kSprocketDiameter = Units.feetToMeters(1.273 / 12.0);
    
    public static final int kLowerLimitSwitchPort = 53;
    public static final int kUpperLimitSwitchPort = 54;

    // Elevator PID Constants
    public static final double kP = 4;
    public static final double kI = 0.0;
    public static final double kD = 0.0; 

    // Elevator FF constants (for rio profiling)
    public static final double kS = 0.1;
    public static final double kV = 21.5;
    public static final double kA = 1;
    public static final double kG = 0.5;

    // Max velocity and acceleration
    public static final double kMaxVelocity = 0.5;
    public static final double kMaxAcceleration = 0.6;

    public static final double kSeekVoltage = 2;

    public static final int kDefaultContinuousCurrentLimit = 35;
    public static final int kDefaultPeakCurrentLimit = 60;

    public static final CANSparkMax.IdleMode kElevatorIdleMode = CANSparkMax.IdleMode.kBrake;
   
}
