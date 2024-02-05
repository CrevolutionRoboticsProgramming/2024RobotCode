package frc.robot.Autos.AutonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CustomWaitCommand extends Command {
    private double waitSeconds;

    public CustomWaitCommand(double waitSeconds) {
        this.waitSeconds = waitSeconds;
    }

    @Override
    public void execute() {
        new WaitCommand(waitSeconds);
    }
}
