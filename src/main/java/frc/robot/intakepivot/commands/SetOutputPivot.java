package frc.robot.intakepivot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.IntakePivotConfig;

class SetOutputPivot extends Command{
    private final IntakePivot pivot;
    private final DoubleSupplier supplier;

    SetOutputPivot(DoubleSupplier outputSupplier) {
        pivot = IntakePivot.getInstance();
        this.supplier = outputSupplier;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        // TODO: safety
        pivot.setOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setOutput(0);
    }
}
