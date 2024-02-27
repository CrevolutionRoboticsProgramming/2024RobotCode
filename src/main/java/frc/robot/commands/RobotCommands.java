package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Shooter.Commands.LoadNote;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.SetElevatorState;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStatePivot;
import frc.robot.intakeroller.IntakeRoller;
import frc.robot.intakeroller.commands.IntakeCommands;
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
                new LoadNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    IntakeCommands.setOutput(() ->-1)
                )
            )
        );
    }
    
    public static Command spitNote() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStatePivot.State.kSpit),
            IntakeCommands.setOutput(() -> 1)
        );
    }
    public static Command primeShooter() {
        return new SequentialCommandGroup(
            handOff(),
            IntakeCommands.setOutput(() -> -1));
            /*Add Output for Shooter Avg */
    }
    // public static Command shootNote() {
    //     return new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //             /*Add Set Angle */
    //             /*Add set RPM for Shooter */
    //         )
    //      /*Run Indexer */
    //     );
    // }
}
