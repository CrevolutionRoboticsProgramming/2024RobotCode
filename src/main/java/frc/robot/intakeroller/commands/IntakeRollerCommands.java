package frc.robot.intakeroller.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class IntakeRollerCommands extends Command {
    public static Command setOutput(DoubleSupplier outputSupplier) {
        return new SetOutputIntakeRoller(outputSupplier);
    }
}
