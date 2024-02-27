package frc.robot.elevator.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.elevator.ElevatorConfig;

public class ElevatorCommands{
    public static Command setState(ElevatorConfig.ElevatorState state) {
        return new SetElevatorState(state);
    }
    public static Command setOuput(DoubleSupplier velocity) {
        return new RunElevatorManual(velocity);
    }
}
