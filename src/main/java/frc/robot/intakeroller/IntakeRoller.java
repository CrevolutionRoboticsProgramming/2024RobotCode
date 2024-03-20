package frc.robot.intakeroller;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    public static class Settings {
        public enum CurrentProfile {
            kIntake(45, 50),
            kOuttake(60, 60);

            final int continuous, peak;
            CurrentProfile(int continuous, int peak) {
                this.continuous = continuous;
                this.peak = peak;
            }

            void apply(CANSparkMax spark) {
                spark.setSmartCurrentLimit(peak, continuous);
            }
        }

        static final int kBeamBreakerId = 3;
    }
    private static IntakeRoller mInstance;
    private static DigitalInput mBeamBreaker;

    private final CANSparkMax mSpark;

    public IntakeRoller() {
        mSpark = new CANSparkMax(IntakeConfig.kIntakeSparkID, MotorType.kBrushless);
        mSpark.setIdleMode(IdleMode.kBrake);
        mBeamBreaker = new DigitalInput(Settings.kBeamBreakerId);

        configureMotor();
    }

    public static IntakeRoller getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeRoller();
        }
        return mInstance;
    }

    public boolean hasNote() {
        return !mBeamBreaker.get();
    }

    public void setOutput(double percentOutput) {
        mSpark.setVoltage(12.0 * percentOutput);
    }

    public double getIntakeRollerPercentOutput() {
        return mSpark.get();
    }

    public void stop() {
        mSpark.set(0);
    }

    public void setCurrentProfile(Settings.CurrentProfile profile) {
        profile.apply(mSpark);
    }

    private void configureMotor() {
        mSpark.setInverted(IntakeConfig.kShooterMotorInverted);
        mSpark.setIdleMode(IntakeConfig.kRollerIdleMode);
    }

    public RelativeEncoder getPivotEncoder() {
        return mSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Percent Output", getIntakeRollerPercentOutput());
        // System.out.println("Intake Roller Current: " + m_Roller.getOutputCurrent());
        SmartDashboard.putBoolean("Intake Has Note", hasNote());
    }
}
