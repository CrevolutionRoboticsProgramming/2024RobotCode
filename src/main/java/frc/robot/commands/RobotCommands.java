package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStatePivot;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.commands.SetStateShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

public class RobotCommands {
    public static Command handOff() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStatePivot.State.kStowed),
//                ElevatorCommands.setState(ElevatorState.kZero),
                ShooterPivotCommands.setState(SetStateShooterPivot.State.kHandoff)
            ),
            new ParallelRaceGroup(
                IndexerCommands.loadNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    IntakeRollerCommands.setOutput(() -> -1)
                )
            )
        );
    }

    public static Command spitNote() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStatePivot.State.kSpit),
            IntakeRollerCommands.setOutput(() -> 1)
        );
    }

    public static Command primeAmpShooter() {
        return new SequentialCommandGroup(
            new ConditionalCommand(null, handOff(), () -> Indexer.getInstance().hasNote()),
            ShooterPivotCommands.setState(SetStateShooterPivot.State.kAmp)
        );
    }

    public static Command primeSpeakerShooter() {
        return new SequentialCommandGroup(
            new ConditionalCommand(null, handOff(), () -> Indexer.getInstance().hasNote()),
            new ParallelCommandGroup(
                ShooterPivotCommands.lockTarget()
                /*Shooter RPM */
            )
        );
    }

    public static Command shootNoteSpeaker() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                /*Add set RPM for Shooter */
                new SequentialCommandGroup(
                    IndexerCommands.setOutput(() -> 1),
                    new WaitCommand(1)
                )
            )
        );
    }

    public static Command shooterNoteAmp() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRotations(4000)),
                new WaitCommand(0.5),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        IndexerCommands.setOutput(() -> 1),
                        new WaitCommand(1)
                    )
                )
            )
        );
    }
}
