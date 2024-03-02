package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;

import java.util.function.DoubleSupplier;

public class SetOutputElevator extends Command {
    private Elevator mElevator;
    private DoubleSupplier mOutputSupplier;

    SetOutputElevator(DoubleSupplier outputSupplier) {
        mElevator = Elevator.getInstance();
        mOutputSupplier = outputSupplier;
        addRequirements(mElevator);
    }

    @Override
    public void execute() {
        mElevator.setOutput(mOutputSupplier.getAsDouble());
    }
}
