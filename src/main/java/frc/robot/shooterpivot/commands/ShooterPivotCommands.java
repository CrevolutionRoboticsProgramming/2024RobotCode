package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class ShooterPivotCommands {
    public static Command setSpeakerAngle(SetAngleShooterPivot.Preset state) {
        return new SetAngleShooterPivot(state.getRotation2d());
    }

    public static Command setSpeakerAngle(Rotation2d angle) {
        return new SetAngleShooterPivot(angle);
    }

    public static Command setAngularVelocity(Supplier<Rotation2d> velocitySupplier, boolean openLoop) {
        return new SetVelocityShooterPivot(velocitySupplier, openLoop);
    }

    // public static Command setState(SetAngleShooterPivot.Preset state) {
    //     return new SetAngleShooterPivot(state, false);
    // }

    public static Command setSpeakerAngle(Supplier<Rotation2d> targetSupplier) {
        return new SetAngleShooterPivot(targetSupplier);
    }

    public static Command setSpeakerAngle(Supplier<Rotation2d> targetSupplier, boolean single) {
        return new SetAngleShooterPivot(targetSupplier, single);
    }

    // public static Command tuneLockSpeaker(Supplier<Rotation2d> targetSupplier) {
    //     return new LockSpeaker(targetSupplier);
    // }

    // public static Command lockSpeaker(Supplier<Rotation2d> angleSupplier) {
    //     return new LockSpeaker(angleSupplier);
    // }

    // public static Command holdState(SetAngleShooterPivot.Preset state) {
    //     return new SetAngleShooterPivot(state, true);
    // }

    // public static Command holdState() {
    //     return new HoldState();
    // }

    // public static Command lockTarget() {
    //     return new LockTarget();
    // }
}
