package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class IndexerCommands {
    public static Command setOutput(DoubleSupplier outputSupplier) {
        return new SetOutputIndexer(outputSupplier);
    }

    public static Command loadNote() {
        return new LoadNote();
    }
}
