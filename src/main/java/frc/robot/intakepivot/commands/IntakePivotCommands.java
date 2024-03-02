package frc.robot.intakepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivotConfig;
import frc.robot.shooterpivot.commands.SetVelocityShooterPivot;

public class IntakePivotCommands {
    public static Command setPivotState(SetStatePivot.State state) {
        return new SetStatePivot(state);
    }
    // public static Command setPivotOutput(DoubleSupplier supplier) {
    //     return new SetOutputPivot(supplier);
    // }
    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        return new SetVelocityIntakePivot(velocitySupplier, openLoop);
    }
}
