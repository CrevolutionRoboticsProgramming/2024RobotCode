package frc.robot.intakeroller.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

public class IntakeCommands extends Command {
    public static Command setOutput(DoubleSupplier outputSupplier) {
        return new SetOutput(outputSupplier);
    }
}
