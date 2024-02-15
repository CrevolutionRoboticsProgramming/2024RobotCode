package frc.robot.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConfig;
import frc.robot.Robot;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Shooter.ShooterIndexer;


public class Shooter extends SubsystemBase {
    private final TalonFX rShooterMotor, lShooterMotor;
    private final ShooterIndexer m_Indexer;

    // private VelocityDutyCycle l_shooterVelocity = ShooterConfig.leftShooterVelocity;
    // private VelocityDutyCycle r_shooterVelocity = ShooterConfig.rightShooterVelocity;
    // private DutyCycleOut l_shooterPercentOutput = ShooterConfig.leftShooterPercentOutput;
    // private DutyCycleOut r_shooterPercentOutput = ShooterConfig.rightShooterPercentOutput;
    private VelocityDutyCycle shooterVelocityOutput = ShooterConfig.shooterVelocityOutput;
    private DutyCycleOut shooterPercentOutput = ShooterConfig.shooterPercentOutput;


    public Shooter() {
        rShooterMotor = new TalonFX(ShooterConfig.kRightShooterID);
        lShooterMotor = new TalonFX(ShooterConfig.kLeftShooterID);
        
        m_Indexer = new ShooterIndexer();
    }

    // public void LeftShooterVelocity(double velocity) {
    //     l_shooterVelocity.Velocity = velocity;
    //     lShooterMotor.setControl(l_shooterVelocity);
    // }

    // public void LeftshooterPercentOutput(double percentOutput) {
    //     l_shooterPercentOutput.Output = percentOutput;
    //     lShooterMotor.setControl(l_shooterPercentOutput);
    // }

    // public void RightShooterVelocity(double velocity) {
    //     r_shooterVelocity.Velocity = velocity;
    //     rShooterMotor.setControl(r_shooterVelocity);
    // }

    // public void RightshooterPercentOutput(double percentOutput) {
    //     r_shooterPercentOutput.Output = percentOutput;
    //     rShooterMotor.setControl(r_shooterPercentOutput);
    // }

    public void shooterPercentOutput(double percentOutput) {
        shooterPercentOutput.Output = percentOutput;
        rShooterMotor.setControl(shooterPercentOutput);
        lShooterMotor.setControl(shooterPercentOutput);
    }

    public void shooterVeclocity(double velocity) {
        shooterVelocityOutput.Velocity = velocity;
        rShooterMotor.setControl(shooterVelocityOutput);
        lShooterMotor.setControl(shooterVelocityOutput);
    }

    public void stop() {
        rShooterMotor.set(0);
        lShooterMotor.set(0);
    }

    public void setProfile(ShooterConfig.ShooterProfile profile) {
        rShooterMotor.getConfigurator().apply(Robot.ctreConfigs.shooterConfig);
        lShooterMotor.getConfigurator().apply(Robot.ctreConfigs.shooterConfig);
    }

    public double[] getShooterPercentOutputs() {
        return new double[]{rShooterMotor.get(), lShooterMotor.get()};
    }

    public double[] getShooterVelocities() {
        return new double[]{rShooterMotor.getVelocity().getValueAsDouble(), lShooterMotor.getVelocity().getValueAsDouble()};
    }

    // public double getRightShooterVelocity() {
    //     return rShooterMotor.getVelocity().getValueAsDouble();
    // }
    // public double getLeftShooterVelocity() {
    //     return lShooterMotor.getVelocity().getValueAsDouble();
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Right Shooter Velocity", getRightShooterVelocity());
        // SmartDashboard.putNumber("Left Shooter Velocity", getLeftShooterVelocity());

        SmartDashboard.putNumberArray("Shooter Percent Output", getShooterPercentOutputs());
        SmartDashboard.putNumberArray("Shooter Velocities", getShooterVelocities());
        SmartDashboard.putNumber("Indexer Velocity", m_Indexer.getIndexerSpeed());
    }
    
}
