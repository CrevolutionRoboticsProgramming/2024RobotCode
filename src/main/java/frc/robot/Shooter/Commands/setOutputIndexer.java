package frc.robot.Shooter.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterIndexer;
import frc.robot.intakeroller.IntakeRoller;
import pabeles.concurrency.IntOperatorTask.Min;

public class setOutputIndexer extends Command {
    private final ShooterIndexer mIndex;
    private final DoubleSupplier supplier;

    public setOutputIndexer(DoubleSupplier supplier) {
        mIndex = ShooterIndexer.getInstance();
        this.supplier = supplier;
        addRequirements(mIndex);
    }

    @Override
    public void execute() {
        mIndex.setIntakeSpeed(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        mIndex.stopIndexer();
    }
}
