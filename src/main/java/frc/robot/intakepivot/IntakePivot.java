package frc.robot.intakepivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

    private static class Settings {
        static final int kSparkID = 21;
        static final boolean kSparkInverted = false;
        static final CANSparkBase.IdleMode kSparkIdleMode = CANSparkBase.IdleMode.kBrake;

        static final boolean kEncoderInverted = false;
        static final double kEncoderZeroOffset = 0;

        // 45 degrees per second
        static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(45);
        static final double kMaxVoltage = 12.0;

        static final double kG = 0.17; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 1.32; // V * sec / rad
        static final double kA = 0.0;  // V * sec^2 / rad
        static final double kP = 0.0;
        static final double kI = 0.0;
        static final double kD = 0.0;
    }

    private static IntakePivot mInstance;
    private final CANSparkMax mSpark;
    private final AbsoluteEncoder encoder;
    private final ArmFeedforward ffContoller;
    private final PIDController pidController;

    public IntakePivot() {
        mSpark = new CANSparkMax(Settings.kSparkID, MotorType.kBrushless) {{
            setInverted(Settings.kSparkInverted);
            setIdleMode(Settings.kSparkIdleMode);
        }};

        encoder = mSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setZeroOffset(IntakePivotConfig.kPivotZeroOffset);
        encoder.setInverted(IntakePivotConfig.kPivotEncoderInverted);

        ffContoller = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
        pidController = new PIDController(Settings.kP, Settings.kI, Settings.kD);
    }

    public static IntakePivot getInstance() {
        if (mInstance == null) {
            mInstance = new IntakePivot();
        }
        return mInstance;
    }

    public void setOutput(double output) {
        mSpark.set(output);
    }

    /**
     * @param angularVelocity in units per second
     */
    public void setVelocity(Rotation2d angularVelocity) {
        final var ffComponent = ffContoller.calculate(getAngle().getRadians(), angularVelocity.getRadians());
        final var pidComponent = pidController.calculate(getAngularVelocity().getRadians(), angularVelocity.getRadians());
        mSpark.setVoltage(ffComponent + pidComponent);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getPosition());
    }

    /**
     * @return velocity in units per seconds
     */
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(encoder.getVelocity() / 60.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Angle (Degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (Degrees / Second)", getAngularVelocity().getDegrees());
    }
}
