package frc.robot.intakepivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivotCommands {
    public static Command setPivotState(SetStateIntakePivot.State state) {
        return new SetStateIntakePivot(state);
    }

    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        return new SetVelocityIntakePivot(velocitySupplier, openLoop);
    }
}
