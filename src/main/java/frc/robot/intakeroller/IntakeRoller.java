package frc.robot.intakeroller;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
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

    public void setOutput(double percentOutput) {
        mSpark.setVoltage(12.0 * percentOutput);
    }

    public double getIntakeRollerPercentOutput() {
        return mSpark.get();
    }

    public void stop() {
        mSpark.set(0);
    }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        mSpark.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    private void configureMotor() {
        mSpark.setInverted(IntakeConfig.kShooterMotorInverted);
        mSpark.setIdleMode(IntakeConfig.kRollerIdleMode);
        mSpark.setSmartCurrentLimit(IntakeConfig.kDefaultContinuousCurrentLimit, IntakeConfig.kDefaultPeakCurrentLimit);
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
