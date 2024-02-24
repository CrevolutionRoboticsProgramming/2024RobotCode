package frc.robot.Elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Elevator.ElevatorConfig;

public class ElevatorCommands{
    public static Command setState(ElevatorConfig.ElevatorState state) {
        return new SetElevatorState(RobotContainer.mElevator, state);
    }
}
