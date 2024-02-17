package frc.robot.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Shooter.ShooterConfig;
import frc.robot.Shooter.ShooterIndexer;

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

    // public void setProfile(ShooterConfig.ShooterProfile profile) {
    //     m_Roller.set
    // }

    public void setCurrentLimit(int continuousLimit, int peakLimit) {
        m_Roller.setSmartCurrentLimit(continuousLimit, peakLimit);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        m_Roller.setIdleMode(mode);
    }

     private void configureMotor() {
        m_Roller.setInverted(ShooterConfig.kPivotMotorInverted);
        m_Roller.setIdleMode(ShooterConfig.kPivotIdleMode);
        m_Roller.setSmartCurrentLimit(ShooterConfig.kDefaultContinuousCurrentLimit, ShooterConfig.kDefaultPeakCurrentLimit);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Percent Output", getIntakeRollerPercentOutput());
    }
}
