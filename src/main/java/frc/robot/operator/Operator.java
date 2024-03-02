package frc.robot.operator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.crevolib.util.ExpCurve;
import frc.crevolib.util.Gamepad;
import frc.robot.commands.RobotCommands;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.indexer.commands.IndexerCommands;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStatePivot;
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

   ExpCurve stickCurve;
   ExpCurve shooterPivotManualCurve;
   ExpCurve intakePivotManualCurve;
   ExpCurve positionTestCurve;
   ExpCurve elevatorCurve;
    private static Operator mInstance;

    ExpCurve shooterManualCurve;

    private Operator() {
        super(Settings.name, Settings.port);

       stickCurve = new ExpCurve(1, 0, 1, Settings.kDeadzone);
       shooterPivotManualCurve = new ExpCurve(1, 0, ShooterPivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
       intakePivotManualCurve = new ExpCurve(1, 0, IntakePivot.Settings.kMaxAngularVelocity.getRadians(), Settings.kDeadzone);
       positionTestCurve = new ExpCurve(1, 20, 15, Settings.kDeadzone);
       elevatorCurve = new ExpCurve(1, 0, 0.8, Settings.kDeadzone);

        shooterManualCurve = new ExpCurve(1, 0, ShooterFlywheel.Settings.kMaxAngularVelocity.getRadians(), 0.05);
    }

    public static Operator getInstance() {
        if (mInstance == null) {
            mInstance = new Operator();
        }
        return mInstance;
    }

    @Override
    public void setupTeleopButtons() {

        //Intake Roller Stuff
         controller.R1().whileTrue(IntakeRollerCommands.setOutput(() -> 1));
         controller.L1().whileTrue(IntakeRollerCommands.setOutput(() -> -1));

        // Shooter Test Commands
        controller.cross().whileTrue(ShooterFlywheelCommands.setAngularVelocity(() -> ShooterFlywheel.Settings.kMaxAngularVelocity));
        controller.square().whileTrue(IndexerCommands.setOutput(() -> -1));

        controller.circle().onTrue(RobotCommands.handOff());

        //Elevator Manual Override
        controller.R2().whileTrue(
            ElevatorCommands.setOutput(() -> elevatorCurve.calculate(controller.getRightY()))
        );

        //Intake Pivot Mnaual Override
        controller.L2().whileTrue(
            IntakePivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
                intakePivotManualCurve.calculate(controller.getLeftX())), 
                false)
        );

        //Shooter Pivot Manual Override
        controller.L2().whileTrue(
            ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
                shooterPivotManualCurve.calculate(controller.getRightX())), 
                false)
        );

        controller.povLeft().onTrue(IntakePivotCommands.setPivotState(SetStatePivot.State.kDeployed));
        controller.povUp().onTrue(IntakePivotCommands.setPivotState(SetStatePivot.State.kSpit));
        controller.povRight().onTrue(IntakePivotCommands.setPivotState(SetStatePivot.State.kStowed));

        //Shooter Test Commands
        // controller.cross().onTrue(ShooterPivotCommands.setState(SetStateShooterPivot.State.kHandoff));
        // controller.triangle().onTrue(ShooterPivotCommands.setState(SetStateShooterPivot.State.kPrime));

        //  controller.R2().whileTrue(ShooterPivotCommands.setAngularVelocity(
        //      () -> Rotation2d.fromRadians(shooterPivotManualCurve.calculate(controller.getRightX())), true
        //  ));

        // controller.R2().negate().and(controller.R1()).whileTrue(
        //     new SetPosition(() -> Rotation2d.fromDegrees(positionTestCurve.calculate(controller.getRightX()))
        // ));
    }

    @Override
    public void setupDisabledButtons() {

    }

    @Override
    public void setupTestButtons() {
        //Shoot Manual
        controller.R2().whileTrue(ShooterFlywheelCommands.setAngularVelocity(
            () -> Rotation2d.fromRadians(shooterManualCurve.calculate(getRightTriggerMagnitude()))
        ));

        // Indexer (Intake & OutTake)
        controller.cross().whileTrue(IndexerCommands.setOutput(() -> 1.0));
        controller.square().whileTrue(IndexerCommands.setOutput(() -> -1.0));
        

        //Elevator Manual Override
        controller.R2().and(rightYTrigger(ThresholdType.ABS_GREATER_THAN, 0.1).whileTrue(
            ElevatorCommands.setOutput(() -> elevatorCurve.calculate(controller.getRightY())))
        );

        //Intake Pivot Mnaual Override
        controller.L2().and(leftXTrigger(ThresholdType.ABS_GREATER_THAN, 0.1).whileTrue(
            IntakePivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
                intakePivotManualCurve.calculate(controller.getLeftX())), 
                false)
            )
        );

        //Shooter Pivot Manual Override
        controller.L2().and(rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.1).whileTrue(
            ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRadians(
                shooterManualCurve.calculate(controller.getRightX())), 
                false)
            )
        );
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
