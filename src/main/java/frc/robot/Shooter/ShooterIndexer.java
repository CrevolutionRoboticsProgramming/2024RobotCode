package frc.robot.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shooter.ShooterConfig.*;
import frc.robot.drivetrain.Drivetrain;

public class ShooterIndexer extends SubsystemBase{
    private final Victor shooterIndex;
    private final DigitalInput beamBreaker;
    private static ShooterIndexer mInstance;

    public ShooterIndexer() {
        shooterIndex = new Victor(ShooterConfig.kIndexVictorID);
        shooterIndex.setInverted(ShooterConfig.kIndexerMotorInverted);

        beamBreaker = new DigitalInput(ShooterConfig.kIndexerBeamBreak);
    }

    public static ShooterIndexer getInstance() {
        if (mInstance == null) {
            mInstance = new ShooterIndexer();
        }
        return mInstance;
    }

    public void setIntakeSpeed(double velocity) {
        shooterIndex.set(velocity);
    }

    public void setOuttakeSpeed(double velocity) {
        shooterIndex.set(velocity);
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
