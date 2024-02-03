package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

    private VelocityDutyCycle shooterVelocity = new VelocityDutyCycle(0);
    private DutyCycleOut shooterPercentOutput = new DutyCycleOut(0);
    private PositionDutyCycle shooterPivotPosition = new PositionDutyCycle(0);
    private DutyCycleOut shooterPivotPercentOutput = new DutyCycleOut(0);

    public Shooter() {

        m_shooter = new TalonFX(Constants.SHOOTER_ID);
        m_shooterPivot = new TalonFX(Constants.SHOOTER_PIVOT_ID);
        m_shooterEncoder = new CANcoder(Constants.SHOOTER_CANCODER_ID);
    }

    public void shooterRun(double velocity) {

        shooterVelocity.Velocity = velocity;
        m_shooter.setControl(shooterVelocity);
    }

    public void shooterPercentOutput(double percentOutput) {

        shooterPercentOutput.Output = percentOutput;
        m_shooter.setControl(shooterPercentOutput);
    }

    public void shooterPivot(double position) {

        shooterPivotPosition.Position = position;
        m_shooter.setControl(shooterPivotPosition);
    }

    public void shooterPivotPercentOutput(double percentOutput) {

        shooterPivotPercentOutput.Output = percentOutput;
        m_shooter.setControl(shooterPercentOutput);
    }

    public void resetShooterEncoder() {

        m_shooter.setPosition(0);

    }

    public void resetShooterPivotEncoder() {

        m_shooterPivot.setPosition(0);

    }

    public void configShooterMotor() {

        m_shooter.getConfigurator().apply(new TalonFXConfiguration());
        m_shooter.getConfigurator().apply(Robot.ctreConfigs.shooterFXConfig);
        resetShooterEncoder();

    }

    public void configShooterPivotMotor() {

        m_shooterPivot.getConfigurator().apply(new TalonFXConfiguration());
        m_shooterPivot.getConfigurator().apply(Robot.ctreConfigs.shooterPivotFXConfig);
        resetShooterPivotEncoder();

    }

    public double getShooterVelocity() {

        return m_shooter.getVelocity().getValueAsDouble();

    }


    public double getShooterEncoderPosition() {
        return m_shooterEncoder.getPosition().getValueAsDouble();

    }

    public enum ShooterState {

        RESET(Constants.SHOOTER_PIVOT_RESET, Constants.SHOOTER_RESET),
        SHOOT(Constants.SHOOTER_PIVOT_SHOOT, Constants.SHOOTER_SHOOT),
        AMP(Constants.SHOOTER_PIVOT_AMP, Constants.SHOOTER_AMP),
        CLIMB(Constants.SHOOTER_PIVOT_CLIMB, Constants.SHOOTER_RESET);

        public int shooterPivot;
        public double shooterVelocity;
        private ShooterState(int shooterPivot, double shooterVelocity) {
            this.shooterPivot = shooterPivot;
            this.shooterVelocity = shooterVelocity;
        }

    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Shooter Pivot Position", getShooterPivotPosition());
        SmartDashboard.putNumber("Shooter Encoder Position", getShooterEncoderPosition());
        SmartDashboard.putNumber("Shooter Pivot Rotor Position", getShooterPivotRotorPosition());

    }
    
}
}
