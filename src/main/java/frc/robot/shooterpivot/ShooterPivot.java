package frc.robot.shooterpivot;

import java.util.Optional;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.vision.Vision;

public class ShooterPivot extends SubsystemBase {
    public static class Settings {
        static final int kSparkId = 24;

        static final double kG = 0.38; // V
        static final double kS = 0.0;  // V / rad
        static final double kV = 2.65; // V * sec / rad
        static final double kA = 0.01; // V * sec^2 / rad

        static final double kPosP = 8.0; // V / rad
        static final double kPosI = 0.0;
        static final double kPosD = 0.0;

        static final double kVelP = 0.0;//1.5;//0.5;
        static final double kVelI = 0.0;
        static final double kVelD = 0.0;

        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromDegrees(320); //120
        public static final Rotation2d kMaxAngularAcceleration = Rotation2d.fromDegrees(300);
        public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(180);
        public static final Rotation2d kMaxAnglePhysical = Rotation2d.fromDegrees(270);

        public static final int KMaxVoltage = 30;

        // kFFAngleOffset is the differnce between our zero point (handoff) and 'true' zero (parallel to the ground)
        private static final Rotation2d kFFAngleOffset = Rotation2d.fromDegrees(30);
    }

    private static ShooterPivot mInstance;

    private CANSparkMax mSpark;
    private final SparkAbsoluteEncoder mPosEncoder;
    private final RelativeEncoder mVelEncoder;
    private final PIDController mPositionPIDController, mVelocityPIDController;
    private final ArmFeedforward mFFController;

    private Rotation2d lastRequestedVelocity;

    private ShooterPivot() {
        mSpark = new CANSparkMax(Settings.kSparkId, CANSparkLowLevel.MotorType.kBrushless) {{
            setIdleMode(IdleMode.kBrake);
            setInverted(true);
            setSmartCurrentLimit(Settings.KMaxVoltage);
        }};

        mVelEncoder = mSpark.getEncoder();

        mPosEncoder = mSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mPosEncoder.setInverted(false);

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

    // public void setShooterPivotIdleMode(CANSparkBase.IdleMode kIdleMode) {
    //     mSpark = new CANSparkMax(Settings.kSparkId, CANSparkLowLevel.MotorType.kBrushless) {{
    //         setIdleMode(kIdleMode);
    //         setInverted(true);
    //         setSmartCurrentLimit(Settings.KMaxVoltage);
    //     }};
    // }

    public void setAngularVelocity(Rotation2d velocity, boolean openLoop) {
        lastRequestedVelocity = velocity;
        final var currentAngle = getAngle();
        final var currentVelocity = getAngularVelocity();
        final var ffComponent = mFFController.calculate(currentAngle.getRadians() - Settings.kFFAngleOffset.getRadians(), velocity.getRadians());
        final var pidComponent = (openLoop) ? 0.0 : mVelocityPIDController.calculate(currentVelocity.getRadians(), velocity.getRadians());
        // System.out.println("ff: " + ffComponent + ", pid: " + pidComponent);
        mSpark.setVoltage(ffComponent + pidComponent);
    }

    public void setAngularVelocity(Rotation2d velocity) {
        setAngularVelocity(velocity, false);
    }

    public double getDistanceFromSpeaker() {
        Pose2d goalPose = new Pose2d();
        final var mPoseEstimator = Vision.PoseEstimator.getInstance();
        final var robotPose = mPoseEstimator.getCurrentPose();
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {
                goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42)), new Rotation2d(0));
            }
            if (ally.get() == Alliance.Red) {
                goalPose = new Pose2d(new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42)), new Rotation2d(Units.degreesToRadians(180)));
            }
        }
        var xDiff = Math.pow((robotPose.getX() - goalPose.getX()), 2);
        var yDiff = Math.pow((robotPose.getY() - goalPose.getY()), 2);
        var diff = Math.sqrt((xDiff + yDiff));
        return diff;
    }

    public void setAngle(Rotation2d angle) {
        // Our target velocity should always be zero when holding an angle, use positional PID to compensate for error
        final var currentAngle = getAngle();
        final var ffComponent = mFFController.calculate(currentAngle.getRadians() - Settings.kFFAngleOffset.getRadians(), 0);
        final var pidComponent = mPositionPIDController.calculate(currentAngle.getRadians(), angle.getRadians());
        mSpark.setVoltage(ffComponent + pidComponent);
    }

    public Rotation2d getAngularVelocity() {
//        return Rotation2d.fromRotations(mPosEncoder.getVelocity());
        return Rotation2d.fromRotations(mVelEncoder.getVelocity()).div(126.0 * 60.0);
    }

    // TODO: Clamp to zero to prevent loop around reading, issue with cumulative error?
    public Rotation2d getAngle() {
        var pos = mPosEncoder.getPosition();
        if (pos > Settings.kMaxAnglePhysical.getRotations()) {
            pos = 0;
        }
        return Rotation2d.fromRotations(pos);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Pivot Angle (degrees)", getAngle().getDegrees());
        SmartDashboard.putNumber("Shooter Pivot Angular Velocity (degrees / sec)", getAngularVelocity().getDegrees());
        if (lastRequestedVelocity != null) {
            SmartDashboard.putNumber("Shooter Pivot Angular Velocity Requested (degrees / sec)", lastRequestedVelocity.getDegrees());
        }
        SmartDashboard.putNumber("Estimated Disp. Speaker-Robot", getDistanceFromSpeaker());
    }
}
