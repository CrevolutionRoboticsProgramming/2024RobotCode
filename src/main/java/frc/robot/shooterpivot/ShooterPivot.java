package frc.robot.shooterpivot;

import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
    public static class Settings {
        static final int kSparkId = 0;

        static final double kG = 0.45; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 2.46; // V * sec / rad
        static final double kA = 0.01; // V * sec^2 / rad

        static final double kPosP = 0.0;
        static final double kPosI = 0.0;
        static final double kPosD = 0.0;

        static final double kVelP = 0.0;
        static final double kVelI = 0.0;
        static final double kVelD = 0.0;

        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(180);
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(90);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(180);
        public static final Rotation2d kMaxAnglePhysical = Rotation2d.fromDegrees(270);

        // kFFAngleOffset is the differnce between our zero point (handoff) and 'true' zero (parallel to the ground)
        private static final Rotation2d kFFAngleOffset = Rotation2d.fromDegrees(30);

        private static final Rotation2d kEncoderOffset = Rotation2d.fromRotations(0.0);
    }

    private static ShooterPivot mInstance;

    private final CANSparkMax mSpark;
    private final AbsoluteEncoder mEncoder;
    private final PIDController mPositionPIDController, mVelocityPIDController;
    private final ArmFeedforward mFFController;

    private ShooterPivot() {
        mSpark = new CANSparkMax(Settings.kSparkId, CANSparkLowLevel.MotorType.kBrushless);
        mSpark.setIdleMode(CANSparkBase.IdleMode.kBrake);

        mEncoder = mSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mEncoder.setZeroOffset(Settings.kEncoderOffset.getRotations());

        mPositionPIDController = new PIDController(Settings.kPosP, Settings.kPosI, Settings.kPosD);
        mVelocityPIDController= new PIDController(Settings.kVelP, Settings.kVelI, Settings.kVelD);
        mFFController = new ArmFeedforward(Settings.kS, Settings.kG, Settings.kV, Settings.kA);
    }

    public static ShooterPivot getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterPivot();
        }
        return mInstance;
    }

    public void setAngularVelocity(Rotation2d velocity, boolean openLoop) {
        final var currentAngle = getAngle();
        final var currentVelocity = getAngularVelocity();
        final var ffComponent = mFFController.calculate(currentAngle.getRadians() + Settings.kFFAngleOffset.getRadians(), velocity.getRadians());
        final var pidComponent = (openLoop) ? 0.0 : mVelocityPIDController.calculate(currentVelocity.getRadians(), velocity.getRadians());
        mSpark.setVoltage(ffComponent + pidComponent);
    }

    public void setAngularVelocity(Rotation2d velocity) {
        setAngularVelocity(velocity, false);
    }

    public void setAngle(Rotation2d angle) {
        // Our target velocity should always be zero when holding an angle, use positional PID to compensate for error
        final var currentAngle = getAngle();
        final var ffComponent = mFFController.calculate(currentAngle.getRadians() + Settings.kFFAngleOffset.getRadians(), 0);
        final var pidComponent = mPositionPIDController.calculate(currentAngle.getRadians(), angle.getRadians());
        mSpark.setVoltage(ffComponent + pidComponent);
    }

    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(mEncoder.getVelocity());
    }

    // TODO: Clamp to zero to prevent loop around reading, issue with cumulative error?
    public Rotation2d getAngle() {
        var pos = mEncoder.getPosition();
        if (pos > Settings.kMaxAnglePhysical.getRotations()) {
            pos = 0;
        }
        return Rotation2d.fromRotations(pos);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Angle (degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (degrees / sec)", getAngularVelocity().getDegrees());
    }
}
