package frc.robot.intakeroller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final CANSparkMax m_Roller;

    public Intake() {
        m_Roller = new CANSparkMax(IntakeConfig.kIntakeSparkID, MotorType.kBrushless);

        configureMotor();
    }

    public void setIntakeRollerPercentOutput(double percentOutput) {
        m_Roller.set(percentOutput);
    }

    public double getIntakeRollerPercentOutput() {
        return m_Roller.get();
    }

    public void stop() {
        m_Roller.set(0);
    }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        m_Roller.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        m_Roller.setIdleMode(mode);
    }

     private void configureMotor() {
        m_Roller.setInverted(IntakeConfig.kShooterMotorInverted);
        m_Roller.setIdleMode(IntakeConfig.kRollerIdleMode);
        m_Roller.setSmartCurrentLimit(IntakeConfig.kDefaultContinuousCurrentLimit, IntakeConfig.kDefaultPeakCurrentLimit);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Percent Output", getIntakeRollerPercentOutput());
    }
}
