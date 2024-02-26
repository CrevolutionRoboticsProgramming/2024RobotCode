package frc.robot.intakepivot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;

public class IntakePivotConfig {
public enum PivotState {
        
        kDeployed(Units.degreesToRadians(60)),
        // kHumanPlayer(Conversions.degreesToRadians(135)),
        kStowed(Units.degreesToRadians(60)),
        kHandoff(Units.degreesToRadians(60)),
        kUnspecified(0);
        
        /**
         * Creates a PivotState with a target in radians
         *
         * @param target angle in radians
         */
        PivotState(double target) {
            this.target = target;
        }
        public final double target;
    }

    public static final int kPivotSparkID = 28;

    public static final boolean kPivotMotorInverted = false;

    public static final CANSparkMax.IdleMode kPivotIdleMode = CANSparkMax.IdleMode.kBrake;

    // Tenstioning Constants
    public static final int kDefaultContinuousCurrentLimit = 35;
    public static final int kDefaultPeakCurrentLimit = 60;

    // Sensor Parameters
    public static final boolean kPivotEncoderInverted = true;
    public static final double kPivotZeroOffset = 0.95;

    // Pivot PID constants
    public static final double kVelP = 0.1;
    public static final double kVelI = 0.0;
    public static final double kVelD = 0.0;

    // Pivot Feedforward Constants (reca.lc)
    public static final double kG = 0.23;
    public static final double kS = 0.0;
    public static final double kV = 2.92;
    public static final double kA = 0.01;
}

