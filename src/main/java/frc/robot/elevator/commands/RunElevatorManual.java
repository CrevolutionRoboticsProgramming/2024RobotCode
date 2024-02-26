package frc.robot.elevator.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;

public class RunElevatorManual extends Command{
    private final DoubleSupplier supplier;
    private final Elevator elevator;

    public RunElevatorManual(Elevator elevator, DoubleSupplier supplier) {
        this.elevator = elevator;
        this.supplier = supplier;

        addRequirements(elevator);
    }


    @Override
    public void initialize() {
        elevator.setState(ElevatorConfig.ElevatorState.kUnspecified);
    }

    @Override
    public void execute() {
        final double output = supplier.getAsDouble();
        final var limitStates = elevator.getLimitStates();
        if (output < 0 && limitStates[0]) {
            elevator.setOutput(0);
        } else if (output > 0 && limitStates[1]) {
            elevator.setOutput(0);
        } else {
            elevator.setOutput(output * 12.0);
        }
     
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setOutput(0);
    }
}
