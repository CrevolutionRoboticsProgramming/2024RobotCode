package frc.robot.commands;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.driver.Driver;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DriveAndAim;
import frc.robot.drivetrain.commands.DriveAndStopAim;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.drivetrain.commands.TurnAngleProfile;
import frc.robot.elevator.Elevator;
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

import com.revrobotics.CANSparkBase;

public class RobotCommands {
    public static Command handOffNote() {
        //ShooterPivot.getInstance().setShooterPivotIdleMode(CANSparkBase.IdleMode.kBrake);
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
                //ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoffClear)
            ),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kHandoff),
            new ParallelRaceGroup(
                IndexerCommands.grabNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    IntakeRollerCommands.setOutput(() -> 0.5)
                )
            ),
            new InstantCommand(() -> System.out.println("handoff complete"))
        );
    }

    public static Command harmonize() {
        //ShooterPivot.getInstance().setShooterPivotIdleMode(CANSparkBase.IdleMode.kCoast);
        return new SequentialCommandGroup(
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command passNote() {
        //ShooterPivot.getInstance().setShooterPivotIdleMode(CANSparkBase.IdleMode.kBrake);
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasFinalNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero),
                    ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kPass),
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.65)
                    )
                )
            ),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command pass() {
        return new ParallelCommandGroup(
            passNote(),
            DrivetrainCommands.holdPassPos()
        );
    }

    public static Command primeShoot() {
        //ShooterPivot.getInstance().setShooterPivotIdleMode(CANSparkBase.IdleMode.kBrake);
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasFinalNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ShooterPivotCommands.setSpeakerAngle(() ->
                    {
                        System.out.println("RUN");
                        return Rotation2d.fromDegrees(
                            ShooterInterpolation.getInstance().getInterpolatedAngle(
                                ShooterPivot.getInstance().getDistanceFromSpeaker()
                            )
                        );
                    }, false
                    ),
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.7),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(1.0)
                    )
                )
            )
        );
    }

    public static Command prime() {
        //DriverXbox.getInstance().autoAim = true;
        return new ParallelCommandGroup(
            primeShoot(),
            //new DriveAndAim()
            DrivetrainCommands.autoLineUp()
        );
    }

    public static Command stopPrime() {
        //DriverXbox.getInstance().autoAim = false;
        return new DriveAndStopAim();
    }

    // Change to Shoot near !!!!!
    public static Command primeCleanUp() {
        return new ParallelCommandGroup(
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kShooterNear),
            ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.7),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(1.0)
            )
        );
    }

    public static Command shootCleanUp() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
            new ParallelCommandGroup(
                IndexerCommands.setOutput(() -> 1.0),
                IntakeRollerCommands.setOutput(() -> 1.0)
            )
        );
    }

    // public static Command shoot() {
    //     final Rotation2d kAllowedError = Rotation2d.fromRotations(5); // 300 RPM
    //     final var leftVel = ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.95);
    //     final var rightVel = ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85);
    //     var left = (Math.abs(leftVel.getRotations()) - 
    //     (Math.abs(ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations()))) < kAllowedError.getRotations();
    //     var right = (Math.abs(rightVel.getRotations()) - 
    //     (Math.abs(ShooterFlywheel.getInstance().getRightFlywheelVelocity().getRotations()))) < kAllowedError.getRotations();
    //     System.out.println("Is Left There??: " + left);
    //     System.out.println("Is Right There??: " + right);
    //     if((left == true) || (right == true)) {
    //         System.out.println("Working???");
    //         return IndexerCommands.setOutput(() ->1.0);
    //     } else {
    //         System.out.println("So sad not Working???");
    //         return IndexerCommands.setOutput(() -> 0.0);
    //     }
    // }

    public static Command spitNote() {
        return new SequentialCommandGroup(
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kSpit),
            new ParallelRaceGroup(
                new WaitCommand(1.0),
                IntakeRollerCommands.setOutput(() -> 1)
            ),
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeSpeaker(SetAngleShooterPivot.Preset state) {
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasFinalNote),
            new ParallelRaceGroup(
                Commands.parallel(
                    ShooterPivotCommands.setSpeakerAngle(state),
                    ShooterFlywheelCommands.setAngularVelocity(() -> switch (state) {
                        case kShooterNear -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.8);
                        default -> ShooterFlywheel.Settings.kMaxAngularVelocity;
                    })
                )
            ),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kZero),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeAmp() {
        //ShooterPivot.getInstance().setShooterPivotIdleMode(CANSparkBase.IdleMode.kBrake);
        return new SequentialCommandGroup(
            new ConditionalCommand(Commands.none(), handOffNote(), Indexer.getInstance()::hasFinalNote),
            Commands.parallel(
                ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kAmp),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kAmp),
                new ParallelRaceGroup(
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.75)),
                    new WaitUntilCommand(() -> !Indexer.getInstance().hasFinalNote())
                )
            )
        );
    }

    public static Command amp() {
        return new SequentialCommandGroup(
            primeAmp(),
            new ConditionalCommand(Commands.none(), primeAmp(), () -> !Indexer.getInstance().hasFinalNote()),
            Commands.parallel(
                ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kZero),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
            )
        );
    }

    public static Command primeClimb() {
        return Commands.parallel(
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kClimb),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kClimb)
        );
    }

    public static Command climb() {
        return Commands.parallel(
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kClimb),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
        );
    }

    public static Command primeTrap() {
        return Commands.sequence(
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kTrap),
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kTrap)
        );
    }

    public static Command trap() {
        return ElevatorCommands.setPosition(SetPositionElevator.Preset.kPostTrap);
    }

    public static Command zero() {
        return Commands.parallel(
            ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kZero),
            new WaitCommand(3)
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
            new ConditionalCommand(Commands.none(), autoHandOffNote(), Indexer.getInstance()::hasFinalNote),
            Commands.parallel(
               // IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed), Do this in path planner
                DrivetrainCommands.autonAutoLineUp(),
                Commands.race(
                    ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.7),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(1.0)
                    ),
                    Commands.sequence(
                        ShooterPivotCommands.setSpeakerAngle(
                            Rotation2d.fromDegrees(
                                ShooterInterpolation.getInstance().getInterpolatedAngle(
                                    ShooterPivot.getInstance().getDistanceFromSpeaker()
                                )
                            )
                        ),
                        new WaitUntilCommand(() -> {
                            final var currentRPS = ShooterFlywheel.getInstance().getRightFlywheelVelocity().getRotations();
                            final var error = Math.abs(currentRPS - (ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.9));
                            // System.out.printf("current: %d, setpoint: %d, err: %d%n", currentRPS, targetRPS, error);
                            return error < 5;
                        }),
                        new WaitCommand(0.2),
                        Commands.race(
                            IndexerCommands.setOutput(() -> 1.0),
                            Commands.waitSeconds(0.15)
                        )
                    )
                )
            )
        );
    }

    public static Command autoConstantlyRPM() {
        return new RepeatCommand(
            ShooterFlywheelCommands.setAngularVelocity(
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.95),
                        () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85)
            )
        );
    }

    public static Command autoPrimeSpeakerAndShoot(SetAngleShooterPivot.Preset state, double targetRPS) {
        return Commands.sequence(
            new ConditionalCommand(Commands.none(), autoHandOffNote(), Indexer.getInstance()::hasFinalNote),
            Commands.parallel(
                Commands.race(
                    ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRotations(targetRPS)),
                    Commands.sequence(
                        ShooterPivotCommands.setSpeakerAngle(state),
                        // new WaitCommand(1),
                        new WaitUntilCommand(() -> {
                            final var currentRPS = ShooterFlywheel.getInstance().getLeftFlywheelVelocity().getRotations();
                            final var error = Math.abs(currentRPS - targetRPS);
                            // System.out.printf("current: %d, setpoint: %d, err: %d%n", currentRPS, targetRPS, error);
                            return error < 5;
                        }),
                        new WaitCommand(0.2),
                        Commands.race(
                            IndexerCommands.setOutput(() -> 1.0),
                            Commands.waitSeconds(0.15)
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

    public static Command autoHandOffNote() {
        return Commands.either(new SequentialCommandGroup(
            new ParallelCommandGroup(
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
                ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
                //ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoffClear)
            ),
            ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kHandoff),
            new ParallelRaceGroup(
                IndexerCommands.grabNote(),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    IntakeRollerCommands.setOutput(() -> 1)
                )
            ),
            new InstantCommand(() -> System.out.println("handoff complete"))),
            Commands.none(), IntakeRoller.getInstance()::hasNote);
    }

    // public static Command autoHandOffNote() {
    //     return new SequentialCommandGroup(
    //         new ConditionalCommand(Commands.none(), runIntake() ,IntakeRoller.getInstance()::hasNote), 
    //         new ParallelCommandGroup(
    //             //IntakeRollerCommands.setOutput(() -> -1.0), //Should pull note in more during handoff
    //             IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kStowed),
    //             pulse(),
    //             ElevatorCommands.setPosition(SetPositionElevator.Preset.kZero)
    //         ),
    //         ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff),
    //         new ParallelRaceGroup(
    //             ShooterFlywheelCommands.setAngularVelocity(
    //                     () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.95),
    //                     () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85)
    //             ),
    //             IndexerCommands.grabNote(),
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.2), //This can be lower after my fix, maybe .2 ish  
    //                 IntakeRollerCommands.setOutput(() -> 1)
    //             )
    //         ),
    //         new InstantCommand(() -> System.out.println("handoff complete"))
    //     );
    // }

    public static Command pulse() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                IntakeRollerCommands.setOutput(() -> -1.0),
                new WaitCommand(0.1)
            ),
            new ParallelRaceGroup(
                IntakeRollerCommands.setOutput(() -> -1.0),
                new WaitCommand(0.1)
            )
        );
    }

    public static Command autoHandoffNote_OPTIMIZED() {
        return new SequentialCommandGroup(
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

    //SATCHIT COMMANDS FOR AUTON OPTIMIZATIONS

    public static Command intakeStowBeamBreakControl() {
        return Commands.either(autoHandOffNote(), runIntake(), IntakeRoller.getInstance()::hasNote);
    }


    public static Command runIntake() {
        return new ParallelCommandGroup(
                IntakeRollerCommands.setOutput(() -> -1.0),
                IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed),
                ShooterPivotCommands.setSpeakerAngle(SetAngleShooterPivot.Preset.kHandoff));
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
