package frc.robot.Elevator;

import frc.robot.CrevoLib.math.Conversions;

public class ElevatorConfig {
    //needs reviewing
    public enum ElevatorState {
        kZero(0),
        kTrap(0.9),
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
    public static final double kSprocketDiameter = Conversions.feetToMeters(1.273 / 12.0);
    
    public static final int kLowerLimitSwitchPort = 53;
    public static final int kUpperLimitSwitchPort = 54;

    //kMaxVelocity and kMaxAcceleration are currently not being used
    public static final double kMaxVelocity = 0.5;
    public static final double kMaxAcceleration = 0.6;

    public static final double kSeekVoltage = 2;

   
}
