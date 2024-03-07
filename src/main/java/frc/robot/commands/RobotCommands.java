package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.SetPositionElevator;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStateIntakePivot;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

import java.util.concurrent.ConcurrentMap;

public class RobotCommands {
    public static Command handOffNote() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
                //ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoffClear)
            ),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff),
            new ParallelRaceGroup(
                IndexerCommands.loadNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    IntakeRollerCommands.setOutput(() -> 1)
                )
            ),
            new InstantCommand(() -> System.out.println("handoff complete"))
        );
    }

    public static Command spitNote() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kSpit),
            new ParallelRaceGroup(
                new WaitCommand(1.0),
                IntakeRollerCommands.setOutput(() -> 1)
            ),
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed)
        );
    }

    public static Command primeSpeaker(SetAngleShooterPivot.Preset state) {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                ShooterPivotCommands.setState(state),
                ShooterFlywheelCommands.setAngularVelocity(() -> switch (state) {
                    case kShooterNear -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.8);
                    default -> ShooterFlywheel.Settings.kMaxAngularVelocity;
                })
            )
        );
    }

    public static Command primeAmp() {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kAmp),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kAmp),
                ShooterFlywheelCommands.setAngularVelocity(
                    () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.4))
            )
        );
    }

    public static Command primeClimb() {
        return Commands.parallel(
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kTrap),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kClimb)
        );
    }

    public static Command primeTrap() {
        return new ParallelCommandGroup(
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kTrap),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kTrap)
        );
    }

    public static Command shootNote(double targetRPM) {
        return Commands.sequence(
            // Wait until shooter RPM is within 250 RPM
            new WaitUntilCommand(() -> Math.abs(ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations() - targetRPM) < 250),
            IndexerCommands.setOutput(() -> 1.0)
        );
    }

    // AUTON COMMANDS
    public static Command autoPrimeSpeakerAndShoot(SetAngleShooterPivot.Preset state, double targetRPM) {
        return Commands.sequence(
            new ConditionalCommand(Commands.none(), autoHandOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed),
                Commands.race(
                    ShooterFlywheelCommands.setAngularVelocity(() -> switch (state) {
                        case kShooterNear -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.8);
                        default -> ShooterFlywheel.Settings.kMaxAngularVelocity;
                    }),
                    Commands.sequence(
                        ShooterPivotCommands.setState(state),
                        new WaitUntilCommand(() -> Math.abs(ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations() - targetRPM) < 250),
                        Commands.race(
                            IndexerCommands.setOutput(() -> 1.0),
                            Commands.waitSeconds(0.5)
                        )
                    )
                )
            )
        );
    }

    public static Command autoShootNote(double targetRPM) {
        return Commands.sequence(
            // Wait until shooter RPM is within 250 RPM
            new WaitUntilCommand(() -> Math.abs(ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations() - targetRPM) < 250),
            IndexerCommands.setOutput(() -> 1.0)
        );
    }

    public static Command autoHandOffNote() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
                //ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoffClear)
            ),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff),
            new ParallelRaceGroup(
                IndexerCommands.loadNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    IntakeRollerCommands.setOutput(() -> 1)
                ),
                Commands.waitSeconds(3)
            ),
            new InstantCommand(() -> System.out.println("handoff complete"))
        );
    }


    /*Not for WoodHaven unless Vision Done */
    // public static Command shootNoteSpeaker() {
    //     return new SequentialCommandGroup(
    //         new ParallelRaceGroup(
    //             /*Add set RPM for Shooter */
    //             new SequentialCommandGroup(
    //                 IndexerCommands.setOutput(() -> 1),
    //                 new WaitCommand(1)
    //             )
    //         )
    //     );
    // }

    // public static Command shooterNoteAmp() {
    //     return new SequentialCommandGroup(
    //         new ParallelRaceGroup(
    //             ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRotations(4000)),
    //             new WaitCommand(0.5),
    //             new SequentialCommandGroup(
    //                 new ParallelRaceGroup(
    //                     IndexerCommands.setOutput(() -> 1),
    //                     new WaitCommand(1)
    //                 )
    //             )
    //         )
    //     );
    // }
}
