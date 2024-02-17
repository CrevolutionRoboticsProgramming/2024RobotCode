package frc.robot.Intake.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConfig;

public class RunIntakeManual extends Command {
    private final DoubleSupplier supplier;
    private final Intake roller;

    public RunIntakeManual(Intake roller, DoubleSupplier supplier) {
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
        // roller.setProfile(IntakeConfig.kDefaultProfile);
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
