package frc.robot.Intake.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Intake.IntakePivot;

public class RunPivotManual extends Command{
    private final IntakePivot pivot;
    private final DoubleSupplier supplier;

    public RunPivotManual(IntakePivot pivot, DoubleSupplier supplier) {
        this.pivot = pivot;
        this.supplier = supplier;

        addRequirements(pivot);
    }

    @Override
    public String getName() {
        return getClass().getName();
    }

    @Override
    public void initialize() {
        pivot.setState(IntakeConfig.PivotState.kUnspecified);
    }

    @Override
    public void execute() {
        final var input = supplier.getAsDouble();
        if (input < 0 && pivot.getAngleRads() < 0) {
            pivot.stop();
        } else {
            pivot.setOutput(input);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setState(pivot.getAngleRads() < 0 ? IntakeConfig.PivotState.kDeployed : IntakeConfig.PivotState.kUnspecified);
        pivot.stop();
    }
}
