package frc.robot.commands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.drivetrain.commands.TurnAngleProfile;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.SetPositionElevator;
import frc.robot.indexer.Indexer;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStateIntakePivot;
import frc.robot.intakeroller.IntakeRoller;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.ShooterInterpolation;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot.Preset;
import frc.robot.vision.Vision;

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
                IndexerCommands.grabNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    IntakeRollerCommands.setOutput(() -> 1)
                )
            ),
            new InstantCommand(() -> System.out.println("handoff complete"))
        );
    }

    public static Command passNote() {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero),
                    ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kPass),
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(1.0)
                    )
                )
            ),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeShoot() {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ShooterPivotCommands.setSpeakerAngle(
                        () -> Rotation2d.fromDegrees(
                            ShooterInterpolation.getInstance().getInterpolatedAngle(
                                ShooterPivot.getInstance().getDistanceFromSpeaker()
                            )
                        )
                    ),
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.75)
                    )
                )
            )
        );
    }

    public static Command prime() {
        return new ParallelCommandGroup(
            primeShoot(),
            DrivetrainCommands.autoLineUp()
        );
    }

    public static Command spitNote() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kSpit),
            new ParallelRaceGroup(
                new WaitCommand(1.0),
                IntakeRollerCommands.setOutput(() -> 1)
            ),
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeSpeaker(SetAngleShooterPivot.Preset state) {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ShooterPivotCommands.setState(state),
                    ShooterFlywheelCommands.setAngularVelocity(() -> switch (state) {
                        case kShooterNear -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.8);
                        default -> ShooterFlywheel.Settings.kMaxAngularVelocity;
                    })
                )
            ),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeAmp() {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kAmp),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kAmp),
                new ParallelRaceGroup(
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.5)),
                    new WaitUntilCommand(() -> !Indexer.getInstance().hasNote())
                )
            )
        );
    }

    public static Command amp() {
        return new SequentialCommandGroup(
            primeAmp(),
            new ConditionalCommand(Commands.none(), primeAmp(), () -> !Indexer.getInstance().hasNote()),
            Commands.parallel(
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kZero),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
            )
        );
    }

    public static Command primeClimb() {
        return Commands.parallel(
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kClimb),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kClimb)
        );
    }

    public static Command climb() {
        return Commands.parallel(
            new SequentialCommandGroup(
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kTrap),
                new WaitCommand(0.5)
            ),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeTrap() {
        return Commands.sequence(
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kTrap),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kTrap)
        );
    }

    public static Command trap() {
        return ElevatorCommands.setPosition(SetPositionElevator.Preset.kPostTrap);
    }

    public static Command zero() {
        return Commands.parallel(
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kZero)
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
    public static Command autoLineupAndShoot() {
        return Commands.sequence(
            new ConditionalCommand(Commands.none(), autoHandOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed),
                DrivetrainCommands.autoLineUp(),
                Commands.race(
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.75)
                    ),
                    Commands.sequence(
                        ShooterPivotCommands.setSpeakerAngle(
                            () -> Rotation2d.fromDegrees(
                                ShooterInterpolation.getInstance().getInterpolatedAngle(
                                    ShooterPivot.getInstance().getDistanceFromSpeaker()
                                )
                            )
                        ),
                        new WaitUntilCommand(() -> {
                            final var currentRPS = ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations();
                            final var error = Math.abs(currentRPS - (ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.8));
                            // System.out.printf("current: %d, setpoint: %d, err: %d%n", currentRPS, targetRPS, error);
                            return error < 6;
                        }),
                        Commands.race(
                            IndexerCommands.setOutput(() -> 1.0),
                            Commands.waitSeconds(0.135)
                        )
                    )
                )
            )
        );
    }

    public static Command autoConstantlyRPM() {
        return new RepeatCommand(
            ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.6),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.5)
            )
        );
    }

    public static Command autoPrimeSpeakerAndShoot(SetAngleShooterPivot.Preset state, double targetRPS) {
        return Commands.sequence(
            new ConditionalCommand(Commands.none(), autoHandOffNote(), Indexer.getInstance()::hasNote),
            Commands.parallel(
                Commands.race(
                    ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRotations(targetRPS)),
                    Commands.sequence(
                        ShooterPivotCommands.setState(state),
                        // new WaitCommand(1),
                        new WaitUntilCommand(() -> {
                            final var currentRPS = ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations();
                            final var error = Math.abs(currentRPS - targetRPS);
                            // System.out.printf("current: %d, setpoint: %d, err: %d%n", currentRPS, targetRPS, error);
                            return error < 6;
                        }),
                        Commands.race(
                            IndexerCommands.setOutput(() -> 1.0),
                            Commands.waitSeconds(0.2)
                        )
                    )
                )
            ),
            new InstantCommand(() -> System.out.println("Auton Prime Speaker & Shooter Complete"))
        );
    }

    public static Command autoShootNote(double targetRPM) {
        return Commands.sequence(
            // Wait until shooter RPM is within 250 RPM
            new WaitUntilCommand(() -> Math.abs(ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations() - (targetRPM) / 60.0) < 4),
            IndexerCommands.setOutput(() -> 1.0)
        );
    }

    // public static Command autoHandOffNote() {
    //     return Commands.either(new SequentialCommandGroup(
    //         new ParallelCommandGroup(
    //             IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
    //             ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
    //             //ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoffClear)
    //         ),
    //         ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff),
    //         new ParallelRaceGroup(
    //             IndexerCommands.loadNote(),
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.25),
    //                 IntakeRollerCommands.setOutput(() -> 1)
    //             )
    //         ),
    //         new InstantCommand(() -> System.out.println("handoff complete"))),
    //         Commands.none(), IntakeRoller.getInstance()::hasNote);
    // }

    public static Command autoHandOffNote() {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), runIntake() ,IntakeRoller.getInstance()::hasNote),
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
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

    public static Command autoHandoffNote_OPTIMIZED() {
        return new SequentialCommandGroup(
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

    //SATCHIT COMMANDS FOR AUTON OPTIMIZATIONS

    public static Command intakeStowBeamBreakControl() {
        return Commands.either(autoHandOffNote(), runIntake(), IntakeRoller.getInstance()::hasNote);
    }


    public static Command runIntake() {
        return new ParallelCommandGroup(
                IntakeRollerCommands.setOutput(() -> -1.0),
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed),
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff));
    }

    public static Command stowIntake() {
        return new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero));
    }


    public static Command autoIntakeAndHandoffSimulatenously() {
        return new ConditionalCommand(new ParallelCommandGroup(stowIntake(), autoHandoffNote_OPTIMIZED()), 
            runIntake(), 
            IntakeRoller.getInstance()::hasNote);
    }

    // public static Command autoIntakeAndHandoffSimulatenously() {
    //     return new ConditionalCommand(
    //         autoHandOffNote(), runIntake(), IntakeRoller.getInstance()::hasNote);
    // }

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
