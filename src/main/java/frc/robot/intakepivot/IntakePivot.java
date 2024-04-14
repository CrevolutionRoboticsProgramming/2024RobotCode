package frc.robot.intakepivot;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intakeroller.IntakeRoller;

public class IntakePivot extends SubsystemBase {
    public static class Settings {
        static final int kSparkID = 21;
        static final boolean kSparkInverted = true;
        static final CANSparkBase.IdleMode kSparkIdleMode = CANSparkBase.IdleMode.kBrake;

        static final boolean kEncoderInverted = true;

        // 45 degrees per second
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(2800); //720
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(2300);
        static final double kMaxVoltage = 12.0;

        static final Rotation2d kFFAngleOffset = Rotation2d.fromDegrees(20); 

        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(185.0); 

        static final double kG = 0.35; // V 0.35
        static final double kS = 0.0;  // V / rad 0.0
        static final double kV = 1.31; // V * sec / rad (1.7) 1.31
        static final double kA = 0.01;  // V * sec^2 / rad 0.01
        static final double kP = 0.0;
        static final double kI = 0.0;
        static final double kD = 0.0;
    }

    private static IntakePivot mInstance;

    private final CANSparkMax mSpark;
    private final AbsoluteEncoder mPosEncoder;
    private final RelativeEncoder mVelEncoder;
    private final ArmFeedforward ffContoller;
    private final PIDController pidController;

    private Rotation2d lastRequestedVelocity;
    private Rotation2d requestedAngle;

    public IntakePivot() {
        mSpark = new CANSparkMax(Settings.kSparkID, MotorType.kBrushless) {{
            setInverted(Settings.kSparkInverted);
            setIdleMode(Settings.kSparkIdleMode);
        }};

//        mPosEncoder = IntakeRoller.getInstance().getPivotEncoder();
        mPosEncoder = mSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mPosEncoder.setInverted(Settings.kEncoderInverted);
        //mPosEncoder.setZeroOffset(0.045);

        mVelEncoder = IntakeRoller.getInstance().getPivotEncoder();
        mVelEncoder.setInverted(Settings.kEncoderInverted);

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

    public void setVelocity(Rotation2d angularVelocity) {
        setVelocity(angularVelocity, false);
    }

    /**
     * @param angularVelocity in units per second
     */
    public void setVelocity(Rotation2d angularVelocity, boolean openLoop) {
        final var ffComponent = ffContoller.calculate(getAngle().minus(Settings.kFFAngleOffset).getRadians(), angularVelocity.getRadians());
        final var pidComponent = (openLoop) ? 0.0 : pidController.calculate(getAngularVelocity().getRadians(), angularVelocity.getRadians());
        mSpark.setVoltage(ffComponent + pidComponent);
        lastRequestedVelocity = angularVelocity;
    }

    public Rotation2d getAngle() {
        final var pos = Rotation2d.fromRotations(mPosEncoder.getPosition());
        if (pos.getDegrees() > 270.0) {
            return Rotation2d.fromRotations(0);
        }
        return pos;
    }

    /**
     * @return velocity in units per seconds
     */
    public Rotation2d getAngularVelocity() {
        // 2/3 factor to account for gearing difference in pos vs vel
        return Rotation2d.fromRotations(mVelEncoder.getVelocity() / 60.0);
    }

    public void setRequestedAngle(Rotation2d angle) {
        requestedAngle = angle;
    }

    public Rotation2d getRequestedAngle() {
        return requestedAngle;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("[IntakePivot] Angle (deg)", getAngle().getDegrees());
        SmartDashboard.putNumber("[IntakePivot] Velocity (deg/s)", getAngularVelocity().getDegrees());
        if (lastRequestedVelocity != null) {
            SmartDashboard.putNumber("[IntakePivot] Req Velocity (deg/s)", lastRequestedVelocity.getDegrees());
        }

        if (requestedAngle != null) {
            SmartDashboard.putNumber("[IntakePivot] Req Angle (deg)", requestedAngle.getDegrees());
        }
    }
}
