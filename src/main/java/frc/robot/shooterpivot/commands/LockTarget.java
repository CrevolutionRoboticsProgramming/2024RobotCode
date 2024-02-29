package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.vision.Vision;

/**
 * LockTarget will continuously update the set position for the shooter pivot such that it is always at the correct
 * angle to fire into the speaker.<br></br>
 * <b>Warning:</b> command never exits
 */
public class LockTarget extends Command {
    private final ShooterPivot pivot;

    LockTarget() {
        pivot = ShooterPivot.getInstance();
    }

    @Override
    public void execute() {
        pivot.setAngle(getAngleToTarget());
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    private Rotation2d getAngleToTarget() {
        return null;
    }
}
