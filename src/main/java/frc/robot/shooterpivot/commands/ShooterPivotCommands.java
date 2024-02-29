package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class ShooterPivotCommands {
    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        return new SetVelocityShooterPivot(velocitySupplier, openLoop);
    }

    public static Command setState(SetStateShooterPivot.State state) {
        return new SetStateShooterPivot(state);
    }

    public static Command holdState() {
        return new HoldState();
    }

    public static Command lockTarget() {
        return new LockTarget();
    }
}
