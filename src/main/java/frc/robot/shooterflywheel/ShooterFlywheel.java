package frc.robot.shooterflywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFlywheel extends SubsystemBase {
    public static class Settings {
        static final int kLeftId = 6;
        static final int kRightId = 9;

        static final Slot0Configs kFlywheelConfigs = new Slot0Configs()
                .withKS(0.0)
                .withKV(0.115)
                .withKP(0.0);

        // 5800 RPM at motor; 11600 RPM at wheels
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromRotations(5800.0 / 60.0);
    }

    private static ShooterFlywheel mInstance;
    private final TalonFX mFalconLeft, mFalconRight;

    private ShooterFlywheel() {
        mFalconLeft = new TalonFX(Settings.kLeftId);
        mFalconLeft.getConfigurator().apply(Settings.kFlywheelConfigs);
        mFalconLeft.setInverted(true);

        mFalconRight = new TalonFX(Settings.kRightId);
        mFalconRight.getConfigurator().apply(Settings.kFlywheelConfigs);
        mFalconRight.setInverted(false);
    }

    // 57 degreedf

    public static ShooterFlywheel getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterFlywheel();
        }
        return mInstance;
    }

    public void setLeftFlywheelVelocity(Rotation2d velocity) {
        mFalconLeft.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public void setRightFlywheelVelocity(Rotation2d velocity) {
        mFalconRight.setControl(new VelocityVoltage(velocity.getRotations()));
    }

    public Rotation2d getLeftFlywheelVelocity() {
        return Rotation2d.fromRotations(mFalconLeft.getVelocity().getValueAsDouble());
    }

    public Rotation2d getRightFlywheelVelocity() {
        return Rotation2d.fromRotations(mFalconRight.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Flywheel Velocity (RPM)", mFalconLeft.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Right Flywheel Velocity (RPM)", mFalconRight.getVelocity().getValueAsDouble() * 60);
    }
}
