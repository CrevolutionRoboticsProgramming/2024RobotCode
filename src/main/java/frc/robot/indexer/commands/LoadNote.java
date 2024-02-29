package frc.robot.indexer.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.indexer.Indexer;

public class LoadNote extends Command{
    private final Indexer indexer;

    public LoadNote() {
        indexer = Indexer.getInstance();
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        indexer.setOutput(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setOutput(0);
    }

    @Override
    public boolean isFinished() {
        return indexer.hasNote();
    }
}
