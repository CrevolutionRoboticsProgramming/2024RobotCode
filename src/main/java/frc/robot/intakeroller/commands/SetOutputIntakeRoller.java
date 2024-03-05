package frc.robot.intakeroller.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakeroller.IntakeRoller;
import frc.robot.intakeroller.IntakeRoller.Settings.CurrentProfile;

class SetOutputIntakeRoller extends Command{
    private final DoubleSupplier supplier;
    private final IntakeRoller roller;

    private CurrentProfile currentProfile = CurrentProfile.kIntake;

    public SetOutputIntakeRoller(DoubleSupplier supplier) {
        roller = IntakeRoller.getInstance();
        this.supplier = supplier;
        addRequirements(roller);
    }

    @Override
    public void execute() {
        final var out = supplier.getAsDouble();
        if (out < 0 && currentProfile != CurrentProfile.kIntake) {
            roller.setCurrentProfile(CurrentProfile.kIntake);
            currentProfile = CurrentProfile.kIntake;
        } else {
            roller.setCurrentProfile(CurrentProfile.kOuttake);
            currentProfile = CurrentProfile.kOuttake;
        }
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
