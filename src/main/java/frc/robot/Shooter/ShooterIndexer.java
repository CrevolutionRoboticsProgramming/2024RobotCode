// package frc.robot.Shooter;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.motorcontrol.Victor;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Shooter.ShooterConfig.*;

// public class ShooterIndexer {
//     private final Victor m_Index;
//     private final DigitalInput BeamBreaker;

//     public ShooterIndexer() {
//         m_Index = new Victor(ShooterConfig.kIndexVictorID);
//         m_Index.setInverted(ShooterConfig.kIndexerMotorInverted);

//         BeamBreaker = new DigitalInput(ShooterConfig.kIndexerBeamBreak);
//     }

//     public void setIntakeSpeed() {
//         m_Index.set(1);
//     }

//     public void setOuttakeSpeed() {
//         m_Index.set(-1);
//     }
    
//     public void stopIndexer() {
//         m_Index.stopMotor();
//     }

//     public Double getIndexerSpeed() {
//         return m_Index.get();
//     }

//     public boolean getBeamBreaker() {
//         return !BeamBreaker.get();
//     }


// }
