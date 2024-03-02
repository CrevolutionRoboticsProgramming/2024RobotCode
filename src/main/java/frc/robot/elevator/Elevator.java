
package frc.robot.elevator;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

        static final int kLowerLimitSwitch = 1;
        static final int kUpperLimitSwitch = 2;

        static final CANSparkBase.IdleMode kIdleMode = CANSparkBase.IdleMode.kCoast;

        public static final double kG = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        static final double kMaxVelocity = 0.0;
        static final double kMaxAcceleration = 0.0;

        static final double kMaxVoltage = 10.0;
        static final double kSprocketDiameter = Units.feetToMeters(1.273 / 12.0);
    }

    private static Elevator mInstance;

    private final CANSparkMax mSparkLeader;
    private final CANSparkMax mSparkFollower;
    private final RelativeEncoder mEncoder;
    private final DigitalInput mLowerLimitSwitch, mUpperLimitSwitch;

    private ElevatorState currentState;

    private Elevator() {
        mSparkLeader = new CANSparkMax(ElevatorConfig.kElevatorSparkID1, MotorType.kBrushless) {{
            setInverted(Settings.kLeftSparkInverted);
            setIdleMode(Settings.kIdleMode);

        }};
        mSparkFollower = new CANSparkMax(ElevatorConfig.kElevatorSparkID2, MotorType.kBrushless) {{
            setInverted(Settings.kRightSparkInverted);
            setIdleMode(Settings.kIdleMode);
            follow(mSparkLeader, true);
        }};
        mLowerLimitSwitch = new DigitalInput(Settings.kLowerLimitSwitch);
        mUpperLimitSwitch = new DigitalInput(Settings.kUpperLimitSwitch);

        //these 2 lines of code need reviewing
        mEncoder = mSparkLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    }

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public void setOutput(double output) {
        setVoltage(output * Settings.kMaxVoltage);
    }

    public void setVoltage(double voltage) {
        mSparkLeader.setVoltage(voltage);
        mSparkFollower.setVoltage(voltage);
    }

    public boolean[] getLimitStates() {
        return new boolean[]{!mLowerLimitSwitch.get(), !mUpperLimitSwitch.get()};
    }

    public double getPosition() {
        return rotationsToMeters(mEncoder.getPosition());
    }

    public void zero() {
        mEncoder.setPosition(0);
    }

    public double getOutputCurrent() {
        return Math.abs(mSparkLeader.getOutputCurrent());
    }

    public void setState(ElevatorState state) {
        currentState = state;
    }

    public ElevatorState getState() {
        return currentState;
    }

    /**
     * @return angular velocity in rads / sec
     */
    public double getVelocityMps() {
        return rotationsToMeters(mEncoder.getVelocity() / 60.0);
    }

    private double rotationsToMeters(double rotations) {
        return Settings.kSprocketDiameter * Math.PI * rotations;
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Elevator Position (m)", getPosition());
        SmartDashboard.putNumber("Elevator Velocity (m/s): ", getVelocityMps());
    }
}



