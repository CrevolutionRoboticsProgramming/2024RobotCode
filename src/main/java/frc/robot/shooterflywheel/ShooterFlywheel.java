package frc.robot.shooterflywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFlywheel extends SubsystemBase {
    public static class Settings {
        static final int kLeftId = 0;
        static final int kRightId = 0;

        static final Slot0Configs kFlywheelConfigs = new Slot0Configs()
            .withKS(0.0)
            .withKV(0.0)
            .withKP(0.0);

        // 12000 RPM
        public static final Rotation2d kMaxAngularVelocity = Rotation2d.fromRotations(12000.0 / 60.0);
    }

    private static ShooterFlywheel mInstance;
    private final TalonFX mFalconLeft, mFalconRight;

    private ShooterFlywheel() {
        mFalconLeft = new TalonFX(Settings.kLeftId);
        mFalconLeft.getConfigurator().apply(Settings.kFlywheelConfigs);

        mFalconRight = new TalonFX(Settings.kRightId);
        mFalconRight.getConfigurator().apply(Settings.kFlywheelConfigs);
    }

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
        return new Rotation2d(mFalconLeft.getVelocity().getValueAsDouble());
    }

    public Rotation2d getRightFlywheelVelocity() {
        return new Rotation2d(mFalconRight.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Flywheel Velocity (RPM)", getLeftFlywheelVelocity().getRotations() / 60.0f);
        SmartDashboard.putNumber("Right Flywheel Velocity (RPM)", getRightFlywheelVelocity().getRotations() / 60.0f);
    }
}
