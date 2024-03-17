
package frc.robot.elevator;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class Elevator extends SubsystemBase {
    public static class Settings {
        static final int kSparkLeaderID = 22;
        static final int kSparkFollowerID = 23;
        static final boolean kLeftSparkInverted = false;
        static final boolean kRightSparkInverted = false;

        static final int kLowerLimitSwitch = 0;
        static final int kUpperLimitSwitch = 2;

        static final CANSparkBase.IdleMode kIdleMode = CANSparkBase.IdleMode.kBrake;

        public static final double kG = 0.01;
        public static final double kS = 0.0;
        public static final double kV = 76.0;
        public static final double kA = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kMaxVelocity = 0.2; //0.15
        public static final double kMaxAcceleration = 0.5;

        public static final double kMaxVoltage = 10.0;
        static final double kSprocketDiameter = Units.inchesToMeters(1.432);

        public static final double kMaxExtension = Units.inchesToMeters(14);
    }

    private static Elevator mInstance;

    private final CANSparkMax mSparkLeader, mSparkFollower;
    private final RelativeEncoder mEncoder;
    private final DigitalInput mLowerLimitSwitch, mUpperLimitSwitch;

    private final ElevatorFeedforward mFFController;
    private final PIDController mPIDController;

    private ElevatorState desiredState;

    private Elevator() {
        mSparkLeader = new CANSparkMax(Settings.kSparkLeaderID, MotorType.kBrushless) {{
            setIdleMode(Settings.kIdleMode);
            setInverted(true);
        }};
        mSparkFollower = new CANSparkMax(ElevatorConfig.kElevatorSparkID2, MotorType.kBrushless) {{
            setIdleMode(Settings.kIdleMode);
            follow(mSparkLeader, true);
        }};
        mLowerLimitSwitch = new DigitalInput(Settings.kLowerLimitSwitch);
        mUpperLimitSwitch = new DigitalInput(Settings.kUpperLimitSwitch);
        mEncoder = mSparkLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        mFFController = new ElevatorFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        mPIDController = new PIDController(Settings.kP, Settings.kI, Settings.kD);
    }

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public void setVelocity(double velocity, boolean openLoop) {
        final var ffComponent = mFFController.calculate(velocity);
        final var pidComponent = (openLoop) ? 0 : mPIDController.calculate(getVelocity(), velocity);
        mSparkLeader.setVoltage(ffComponent + pidComponent);
    }

    public boolean getLowerLimitState() {
        return mLowerLimitSwitch.get();
    }

    public boolean getUpperLimitState() {
        return mUpperLimitSwitch.get();
    }

    public double getPosition() {
        return rotationsToMeters(mEncoder.getPosition());
    }

    /**
     * @return linear velocity in meters per second
     */
    public double getVelocity() {
        return rotationsToMeters(mEncoder.getVelocity() / 60.0);
    }

    public void resetEncoder() {
        mEncoder.setPosition(0);
    }

    private double rotationsToMeters(double rotations) {
        return Settings.kSprocketDiameter * Math.PI * rotations;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (m)", getPosition());
        SmartDashboard.putNumber("Elevator Velocity (ms^-1): ", getVelocity());

        if (getLowerLimitState()) {
            resetEncoder();
        }
    }
}



