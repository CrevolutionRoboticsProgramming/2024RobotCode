// package frc.robot.Shooter;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.AbsoluteEncoder;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Shooter.ShooterConfig;
// import frc.robot.Robot;
// import frc.robot.Intake.IntakeConfig;
// import frc.robot.Shooter.ShooterIndexer;


// public class Shooter extends SubsystemBase {
//     private final TalonFX RShooter, LShooter;
//     private final double LeftToRightDiff;
//     private final ShooterIndexer m_Index;

//     private VelocityDutyCycle L_shooterVelocity = ShooterConfig.LeftShooterVelocity;
//     private VelocityDutyCycle R_shooterVelocity = ShooterConfig.RightShooterVelocity;
//     private DutyCycleOut L_shooterPercentOutput = ShooterConfig.LeftShooterPercentOutput;
//     private DutyCycleOut R_shooterPercentOutput = ShooterConfig.RightShooterPercentOutput;


//     public Shooter() {
//         RShooter = new TalonFX(ShooterConfig.kRightShooterSparkID, "Canivore");
//         LShooter = new TalonFX(ShooterConfig.kLeftShooterSparkID, "Canivore");

//         m_Index = new ShooterIndexer();

//         LeftToRightDiff = ShooterConfig.LeftToRightDiff;
//     }

//     public void LeftShooterVelocity(double velocity) {
//         L_shooterVelocity.Velocity = velocity;
//         LShooter.setControl(L_shooterVelocity);
//     }

//     public void LeftshooterPercentOutput(double percentOutput) {
//         L_shooterPercentOutput.Output = percentOutput;
//         LShooter.setControl(L_shooterPercentOutput);
//     }

//     public void RightShooterVelocity(double velocity) {
//         R_shooterVelocity.Velocity = velocity;
//         RShooter.setControl(R_shooterVelocity);
//     }

//     public void RightshooterPercentOutput(double percentOutput) {
//         R_shooterPercentOutput.Output = percentOutput;
//         RShooter.setControl(R_shooterPercentOutput);
//     }

//     public void stop() {
//         RShooter.set(0);
//         LShooter.set(0);
//     }

//     public void setProfile(ShooterConfig.ShooterProfile profile) {
//         RShooter.getConfigurator().apply(Robot.ctreConfigs.shooterConfig);
//         LShooter.getConfigurator().apply(Robot.ctreConfigs.shooterConfig);
//     }

//     public double getRightShooterVelocity() {
//         return RShooter.getVelocity().getValueAsDouble();
//     }
//     public double getLeftShooterVelocity() {
//         return LShooter.getVelocity().getValueAsDouble();
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Right Shooter Velocity", getRightShooterVelocity());
//         SmartDashboard.putNumber("Left Shooter Velocity", getLeftShooterVelocity());
//         SmartDashboard.putNumber("Indexer Velocity", m_Index.getIndexerSpeed());
//     }
    
// }
