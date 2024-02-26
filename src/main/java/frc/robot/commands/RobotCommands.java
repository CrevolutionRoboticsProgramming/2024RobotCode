package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.intakepivot.commands.SetStatePivot;
import frc.robot.intakepivot.IntakePivotConfig.PivotState;

public class RobotCommands {
    public static Command handOff() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetStatePivot(SetStatePivot.State.kDeployed),
                new SetElevatorState(ElevatorState.kZero)
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
