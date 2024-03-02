package frc.robot.operator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

public class Operator extends Gamepad {
    private static class Settings {
        static final int port = 1;
        static final String name = "operator";

        static final double kDeadzone = 0.1;
    }

//    ExpCurve stickCurve;
//    ExpCurve shooterPivotManualCurve;
//    ExpCurve intakePivotManualCurve;
//    ExpCurve positionTestCurve;
//    ExpCurve elevatorCurve;
    private static Operator mInstance;

    ExpCurve shooterManualCurve;

    private Operator() {
        super(Settings.name, Settings.port);

//        stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
//        shooterPivotManualCurve = new ExpCurve(1, 0, ShooterPivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
//        intakePivotManualCurve = new ExpCurve(1, 0, IntakePivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
//        positionTestCurve = new ExpCurve(1, 20, 15, Settings.kDeadzone);
//        elevatorCurve = new ExpCurve(1, 0, 0.8, Settings.kDeadzone);

        shooterManualCurve = new ExpCurve(1, 0, ShooterFlywheel.Settings.kMaxAngularVelocity.getRadians(), 0.05);
    }

        if (mInstance == null) {
    public static Operator getInstance() {
            mInstance = new Operator();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {
        // Shooter Commands

        //Intake Test Commands
        // controller.R2().whileTrue(IntakePivotCommands.setAngularVelocity(() -> Rotationd.fromDegrees(45), false));
        // controller.L2().whileTrue(IntakePivotCommands.setAngularVelocity(() -> Rotation2d.fromDegrees(-45), false));

         leftTriggerOnly().and(leftYTrigger(ThresholdType.ABS_GREATER_THAN, 0.1)).whileTrue(IntakePivotCommands.setAngularVelocity(
             () -> Rotation2d.fromRadians(intakePivotManualCurve.calculate(controller.getLeftY())), false
         ));

         controller.R1().whileTrue(IntakeRollerCommands.setOutput(() -> 1));
         controller.L1().whileTrue(IntakeRollerCommands.setOutput(() -> -1));

        //Shooter Test Commands
        // controller.cross().onTrue(ShooterPivotCommands.setState(SetStateShooterPivot.State.kHandoff));
        // controller.triangle().onTrue(ShooterPivotCommands.setState(SetStateShooterPivot.State.kPrime));

         controller.R2().whileTrue(ShooterPivotCommands.setAngularVelocity(
             () -> Rotation2d.fromRadians(shooterPivotManualCurve.calculate(controller.getRightX())), true
         ));

        // controller.R2().negate().and(controller.R1()).whileTrue(
        //     new SetPosition(() -> Rotation2d.fromDegrees(positionTestCurve.calculate(controller.getRightX()))
        // ));

        //Elevator Test Commands
        

        // Eleveator set State
        // controller.triangle().onTrue(ElevatorCommands.setState(ElevatorState.kHigh));
        // controller.cross().onTrue(ElevatorCommands.setState(ElevatorState.kZero));

        // Manual Overrides
        /*Intake Manual Override */
        //leftTriggerOnly().and(rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.15).whileTrue(IntakePivotCommands.setPivotOutput(controller::getR2Axis)));

        /*Shooter Manual Override (Needs to change from Intake to Shooter) */
        // leftTriggerOnly().and(leftXTrigger(ThresholdType.ABS_GREATER_THAN, Settings.kDeadzone).whileTrue(
        //     ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(shooterPivotManualCurve.calculate(controller.getLeftX())), true)
        // ));
        // controller.R2().whileTrue(
        //     ShooterFlywheelCommands.setAngularVelocity(() -> Rotation2d.fromRadians(shooterFlywheelManualCurve.calculate(controller.getR2Axis())))
        // );

        /*Elevator Manual Override */
//        controller.L2().whileTrue(ElevatorCommands.setOutput(() -> elevatorCurve.calculate(controller.getLeftY())));
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {
        controller.R2().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
            () -> Rotation2d.fromRadians(shooterManualCurve.calculate(getRightTriggerMagnitude()))
        ));
        controller.cross().whileTrue(IndexerCommands.setOutput(() -> 1.0));
        controller.square().whileTrue(IndexerCommands.setOutput(() -> -1.0));
        controller.R2().and(leftYTrigger(ThresholdType.ABS_GREATER_THAN, 0.1).whileTrue(ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(shooterManualCurve.calculate(controller.getRightY())), false)));


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
