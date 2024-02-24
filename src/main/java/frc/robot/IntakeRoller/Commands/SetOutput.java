package frc.robot.IntakeRoller.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntakeRoller.Intake;
import frc.robot.IntakeRoller.IntakeConfig;

public class SetOutput extends Command{
    private final DoubleSupplier supplier;
    private final Intake roller;

    public SetOutput(Intake roller, DoubleSupplier supplier) {
        this.supplier = supplier;
        this.roller = roller;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "Intake.RunIntakeManual";
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        roller.setIntakeRollerPercentOutput(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        roller.setIntakeRollerPercentOutput(0);
    }
}
