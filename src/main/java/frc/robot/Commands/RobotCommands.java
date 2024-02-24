package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Elevator.ElevatorConfig.ElevatorState;
import frc.robot.Elevator.commands.SetElevatorState;
import frc.robot.IntakePivot.Commands.SetPivotState;
import frc.robot.IntakePivot.IntakePivotConfig.PivotState;

public class RobotCommands {
    public static Command handOff() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetPivotState(RobotContainer.intakePivot, PivotState.kHandoff),
                new SetElevatorState(RobotContainer.mElevator, ElevatorState.kZero)
                /*Add setShooterState */
            ),
            new ParallelRaceGroup(
                /*new laodIndex Command */ 
                new SequentialCommandGroup(
                    new WaitCommand(0.5)
                    /*new OutTake */
                )
            )
        );
    }
}
