package frc.robot.indexer.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.Indexer;

public class SetOutputIndexer extends Command {
    private final Indexer indexer;
    private final DoubleSupplier supplier;

    public SetOutputIndexer(DoubleSupplier supplier) {
        indexer = Indexer.getInstance();
        this.supplier = supplier;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setOutput(0);
    }
}
