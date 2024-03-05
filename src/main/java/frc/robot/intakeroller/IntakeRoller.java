package frc.robot.intakeroller;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    public static class Settings {
        public enum CurrentProfile {
            kIntake(40, 45),
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

    }
    private static IntakeRoller mInstance;

    private final CANSparkMax mSpark;

    public IntakeRoller() {
        mSpark = new CANSparkMax(IntakeConfig.kIntakeSparkID, MotorType.kBrushless);
        configureMotor();
    }

    public static IntakeRoller getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeRoller();
        }
        return mInstance;
    }

    public void setCurrentProfile() {

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
    }
}
