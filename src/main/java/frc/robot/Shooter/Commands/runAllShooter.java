package frc.robot.Shooter.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;

public class runAllShooter extends Command {
    private final DoubleSupplier supplier;
    private final Shooter m_shooter;

    public runAllShooter(Shooter shooter, DoubleSupplier supplier) {
        this.m_shooter = shooter;
        this.supplier = supplier;
    
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
      m_shooter.setShooterPercentOutput(1);
    }

    @Override
    public void execute() {
      
    }
    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
    }
}
