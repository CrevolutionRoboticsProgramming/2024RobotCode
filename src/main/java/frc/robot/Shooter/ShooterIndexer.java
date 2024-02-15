package frc.robot.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Shooter.ShooterConfig.*;

public class ShooterIndexer {
    private final Victor shooterIndex;
    private final DigitalInput beamBreaker;

    public ShooterIndexer() {
        shooterIndex = new Victor(ShooterConfig.kIndexVictorID);
        shooterIndex.setInverted(ShooterConfig.kIndexerMotorInverted);

        beamBreaker = new DigitalInput(ShooterConfig.kIndexerBeamBreak);
    }

    public void setIntakeSpeed() {
        shooterIndex.set(1);
    }

    public void setOuttakeSpeed() {
        shooterIndex.set(-1);
    }
    
    public void stopIndexer() {
        shooterIndex.stopMotor();
    }

    public Double getIndexerSpeed() {
        return shooterIndex.get();
    }

    public boolean getBeamBreaker() {
        return !beamBreaker.get();
    }


}
