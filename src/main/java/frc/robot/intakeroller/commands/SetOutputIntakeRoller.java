package frc.robot.intakeroller.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakeroller.IntakeRoller;

class SetOutputIntakeRoller extends Command{
    private final DoubleSupplier supplier;
    private final IntakeRoller roller;

    public SetOutputIntakeRoller(DoubleSupplier supplier) {
        roller = IntakeRoller.getInstance();
        this.supplier = supplier;
        addRequirements(roller);
    }

    @Override
    public void execute() {
        roller.setOutput(supplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        roller.setOutput(0);
    }
}