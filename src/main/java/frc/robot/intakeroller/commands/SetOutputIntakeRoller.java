package frc.robot.intakeroller.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.driver.Driver;
import frc.robot.driver.DriverXbox;
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

        if(roller.hasNote()) {
            DriverXbox.getInstance().controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // return roller.hasNote();
    }

    @Override
    public void end(boolean interrupted) {
        roller.setOutput(0);
        DriverXbox.getInstance().controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}
