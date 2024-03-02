package frc.robot.shooterpivot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooterpivot.ShooterPivot;

public class SetPosition extends Command {
    private ShooterPivot m_Pivot;
    private Supplier<Rotation2d> positionSupplier;

    public SetPosition(Supplier<Rotation2d> positionSupplier) {
        m_Pivot = ShooterPivot.getInstance();
        this.positionSupplier = positionSupplier;

        addRequirements(m_Pivot);
    }

    @Override
    public void initialize() {
        System.out.println("[shooterpivot] init set pos");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[shooterpivot] end set pos");
    }

    @Override
    public void execute() {
        m_Pivot.setAngle(positionSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
