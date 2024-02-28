package frc.robot.operator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevator.ElevatorConfig.ElevatorState;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakeroller.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

public class Operator extends Gamepad{
    private static class Settings {
        static final int port = 1;
        static final String name = "operator";

        static final double kDeadzone = 0.1;
    }

    ExpCurve stickCurve;
    ExpCurve shooterPivotManualCurve;
    ExpCurve shooterFlywheelManualCurve;
    private static Operator mInstance;

    private Operator() {
        super(Settings.name, Settings.port);

        stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
        shooterPivotManualCurve = new ExpCurve(1, 0, ShooterPivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
        shooterPivotManualCurve = new ExpCurve(1, 0, ShooterFlywheel.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
    }

    public static Operator getInstance() {
        if (mInstance == null) {
            mInstance = new Operator();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        // Eleveator set State
        controller.triangle().onTrue(ElevatorCommands.setState(ElevatorState.kHigh));
        controller.cross().onTrue(ElevatorCommands.setState(ElevatorState.kZero));

        // Manual Overrides
        /*Intake Manual Override */
        //leftTriggerOnly().and(rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.15).whileTrue(IntakePivotCommands.setPivotOutput(controller::getR2Axis)));

        /*Shooter Manual Override (Needs to change from Intake to Shooter) */
        leftTriggerOnly().and(leftXTrigger(ThresholdType.ABS_GREATER_THAN, Settings.kDeadzone).whileTrue(
            ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(shooterPivotManualCurve.calculate(controller.getLeftX())), true)
        ));
        controller.R2().whileTrue(
            ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRadians(shooterFlywheelManualCurve.calculate(controller.getR2Axis())))
        );

        /*Elevator Manual Override */
        leftBumperOnly().and(leftYTrigger(ThresholdType.ABS_GREATER_THAN, 0.15).whileFalse(ElevatorCommands.setOuput(controller::getL2Axis)));
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
