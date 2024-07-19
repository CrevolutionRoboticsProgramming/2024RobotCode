package frc.robot.operator;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.crevolib.util.XboxGamepad;
import frc.robot.Robot;
import frc.robot.commands.RobotCommands;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.elevator.commands.SetPositionElevator;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;
import frc.robot.vision.Vision;

public class OperatorXbox extends XboxGamepad {
    private static class Settings {
        static final int port = 1;
        static final String name = "operator";

        static final double kDeadzone = 0.1;
    }

    ExpCurve stickCurve;
    ExpCurve shooterPivotManualCurve;
    ExpCurve intakePivotManualCurve;
    ExpCurve positionTestCurve;
    ExpCurve elevatorCurve;
    private static OperatorXbox mInstance;

    ExpCurve shooterManualCurve;

    private OperatorXbox() {
        super(Settings.name, Settings.port);

        stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
        shooterPivotManualCurve = new ExpCurve(1, 0, ShooterPivot.Settings.kMaxAngularVelocity.getRadians()*0.3, Settings.kDeadzone);
        intakePivotManualCurve = new ExpCurve(1, 0, IntakePivot.Settings.kMaxAngularVelocity.getRadians()*0.5, Settings.kDeadzone);
        positionTestCurve = new ExpCurve(1, 20, 15, Settings.kDeadzone);
        elevatorCurve = new ExpCurve(1, 0, Elevator.Settings.kMaxVelocity, Settings.kDeadzone);

        shooterManualCurve = new ExpCurve(1, 0, ShooterFlywheel.Settings.kMaxAngularVelocity.getRadians(), 0.05);
    }

    public static OperatorXbox getInstance() {
        if (mInstance == null) {
            mInstance = new OperatorXbox();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        // Shooter Commands
        controller.a().whileTrue(RobotCommands.handOffNote());

        controller.x().whileTrue(RobotCommands.pass());
        controller.b().whileTrue(RobotCommands.amp());

        controller.y().whileTrue(new SequentialCommandGroup(
            IndexerCommands.unJamNote(),
            IndexerCommands.grabNote())
        );

        // controller.triangle().whileTrue(new SequentialCommandGroup(
        //     IndexerCommands.unJamNote(),
        //     RobotCommands.primeSpeaker(SetAngleShooterPivot.Preset.kShooterNear))
        // );

        controller.rightBumper().whileTrue(IndexerCommands.setOutput(() -> 1.0));
        controller.leftTrigger().and(controller.rightBumper()).whileTrue(RobotCommands.shootCleanUp());

        controller.rightStick().onTrue(RobotCommands.zero());
        controller.leftStick().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
            () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85),
            () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.75)
        ));

        // ONlY For Testing
        // controller.square().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
        //     () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.85),
        //     () -> ShooterFlywheel.Settings.kMaxAngularVelocity.times(0.6)
        // ));

        // Trap Command
        controller.leftBumper().whileTrue(IndexerCommands.setOutput(() -> -1.0));
        controller.leftBumper().whileTrue(ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRotations(75)));

        controller.povUp().whileTrue(RobotCommands.primeClimb());
        controller.povDown().whileTrue(RobotCommands.climb());
        controller.povRight().whileTrue(RobotCommands.primeTrap());
        controller.povLeft().whileTrue(RobotCommands.trap());

        //Elevator Manual Override
        controller.rightTrigger().whileTrue( Commands.sequence(
            // Ensure manual override doesn't overextend height extension
            new ConditionalCommand(
                ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kClimb),
                Commands.none(),
                () -> elevatorCurve.calculate(-controller.getRightY()) > 0 && ShooterPivot.getInstance().getAngle().getDegrees() > SetAngleShooterPivot.Preset.kClimb.getDegrees()
            ),
            ElevatorCommands.setVelocity(() -> elevatorCurve.calculate(-controller.getRightY()))
        ));

        //Intake Pivot Mnaual Override
        // controller.leftTrigger().whileTrue(
        //     IntakePivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
        //             intakePivotManualCurve.calculate(controller.getLeftX())),
        //         false)
        // );

        // //Shooter Pivot Manual Override
        // controller.leftTrigger().whileTrue(
        //     ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
        //             shooterPivotManualCurve.calculate(controller.getRightX())),
        //         false)
        // );
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {
    }



    public Translation2d getDriveTranslation() {
        final var xComponent = stickCurve.calculate(controller.getLeftX());
        final var yComponent = stickCurve.calculate(controller.getLeftY());
        // Components are reversed because field coordinates are opposite of joystick coordinates
        return new Translation2d(yComponent, xComponent);
    }

    public double getDriveRotation() {
        return -stickCurve.calculate(controller.getRightX());
    }
}
