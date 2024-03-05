package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.ElevatorConfig;

import java.util.function.DoubleSupplier;

public class ElevatorCommands{
    public static Command setVelocity(DoubleSupplier velocitySupplier) {
        return new SetVelocityElevator(velocitySupplier, false, false);
    }

    public static Command setState(SetPositionElevator.Preset state) {
        return new SetPositionElevator(state);
    }
}
