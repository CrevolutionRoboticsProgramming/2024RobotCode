package frc.robot.Autos.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class TurnInPlaceCommand extends Command{
    private double angle;

    public TurnInPlaceCommand(double angle) {
        this.angle = angle;
    }

    @Override
    public void execute() {
    
    }


    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
