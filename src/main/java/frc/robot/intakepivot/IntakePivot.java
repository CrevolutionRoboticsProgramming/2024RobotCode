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
        static final CANSparkBase.IdleMode kSparkIdleMode = CANSparkBase.IdleMode.kCoast;

        static final boolean kEncoderInverted = false;
        static final double kEncoderZeroOffset = 0;

        // 45 degrees per second
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(180);
        static final double kMaxVoltage = 12.0;

        static final Rotation2d kFFAngleOffset = Rotation2d.fromDegrees(20);

        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(188.0);

        static final double kG = 0.35; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 1.7; // V * sec / rad
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
        

//        encoder = mSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        encoder = IntakeRoller.getInstance().m_Roller.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
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

    public void setVelocity(Rotation2d angularVelocity) {
        setVelocity(angularVelocity, false);
    }
    /**
     * @param angularVelocity in units per second
     */
    public void setVelocity(Rotation2d angularVelocity, boolean openLoop) {
        final var ffComponent = ffContoller.calculate(getAngle().minus(Settings.kFFAngleOffset).getRadians(), angularVelocity.getRadians());
        final var pidComponent = (openLoop) ? 0.0 : pidController.calculate(getAngularVelocity().getRadians(), angularVelocity.getRadians());
//        System.out.println("err: " + (angularVelocity.minus(getAngularVelocity()).getDegrees()));
        mSpark.setVoltage(ffComponent + pidComponent);

        // System.out.println("FF Component: " + ffComponent);
        // System.out.println("Intake Pivot Current: " + mSpark.getOutputCurrent());
    }

    public Rotation2d getAngle() {
        final var pos = Rotation2d.fromRotations(encoder.getPosition()* (2.0/3.0));
        if (pos.getDegrees() > 200.0) {
            return Rotation2d.fromRotations(0);
        }
        return pos;
    }

    /**
     * @return velocity in units per seconds
     */
    public Rotation2d getAngularVelocity() {
        return Rotation2d.fromRotations(encoder.getVelocity() / 60.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Pivot Angle (Degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Intake Pivot Angular Velocity (Degrees / Second)", getAngularVelocity().getDegrees());
        SmartDashboard.putNumber("Intake Pivot Angle (Raw)", encoder.getPosition());
    }
}
