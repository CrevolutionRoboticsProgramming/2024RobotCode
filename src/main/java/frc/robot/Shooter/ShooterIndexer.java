// package frc.robot.Shooter;

// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Shooter.ShooterConfig.*;

// public class ShooterIndexer {
//     private final TalonFX Index;
//     private final DigitalInput BeamBreaker;

//     public ShooterIndexer() {
//         Index = new TalonFX(ShooterConfig.kIndexTalonID, "Canivore");
//         Index.setInverted(ShooterConfig.kIndexerMotorInverted);

//         BeamBreaker = new DigitalInput(ShooterConfig.kIndexerBeamBreak);
//     }

//     public void setIntakeSpeed() {
//         Index.set(1);
//     }

//     public void setOuttakeSpeed() {
//         Index.set(-1);
//     }
    
//     public void stopIndexer() {
//         Index.set(0);
//     }

//     public Double getIndexerSpeed() {
//         return Index.getVelocity().getValueAsDouble();
//     }

//     public boolean getBeamBreaker() {
//         return !BeamBreaker.get();
//     }


// }
