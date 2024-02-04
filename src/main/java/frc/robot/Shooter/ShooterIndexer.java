// package frc.robot.Shooter;

// import com.ctre.phoenix6.hardware.TalonFX;
// import frc.robot.Shooter.ShooterConfig.*;

// public class ShooterIndexer {
//     private final TalonFX Index;
//     public ShooterIndexer() {
//         Index = new TalonFX(ShooterConfig.kIndexTalonID, "Canivore");
//         Index.setInverted(ShooterConfig.kIndexerMotorInverted);
//     }

//     public void setIntakeSpeed() {
//         Index.set(1);
//     }

//     public void setOuttakeSpeed() {
//         Index.set(-1);
//     }

//     public Double getIndexerSpeed() {
//         return Index.getVelocity().getValueAsDouble();
//     }


// }
