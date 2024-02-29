package frc.robot.shooterpivot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;

public class HoldState extends Command {
    private final ShooterPivot pivot;
    private Rotation2d targetAngle;

    HoldState() {
        pivot = ShooterPivot.getInstance();
    }

    @Override
    public void initialize() {
        targetAngle = pivot.getAngle();
    }

    @Override
    public void execute() {
        pivot.setAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
