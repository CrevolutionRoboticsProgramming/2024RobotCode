package frc.robot.LEDs.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommands extends Command{
    public static Command setLEDColor() {
        return new setLEDColor();
    }
}
