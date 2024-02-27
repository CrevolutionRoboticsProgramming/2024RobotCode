package frc.robot.Shooter.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.ShooterIndexer;
import frc.robot.Shooter.ShooterIndexer.*;

public class LoadNote extends Command{
    private final ShooterIndexer mIndex;

    public LoadNote() {
        mIndex = ShooterIndexer.getInstance();

        addRequirements(mIndex);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mIndex.setIntakeSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        mIndex.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return mIndex.getBeamBreaker();
    }
}
