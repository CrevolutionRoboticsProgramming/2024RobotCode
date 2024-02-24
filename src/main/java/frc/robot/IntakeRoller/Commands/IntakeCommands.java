package frc.robot.IntakeRoller.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.IntakeRoller.Intake;
import frc.robot.IntakeRoller.IntakeConfig;

public class IntakeCommands extends Command {
    @Deprecated
    public static Command runIntakeRollerManual(double supplier) {
        return new InstantCommand(() -> RobotContainer.intakeRoller.setIntakeRollerPercentOutput(supplier));
    }

    @Deprecated
    public static Command runIntakeRoller(double output) {
        return new InstantCommand(() -> RobotContainer.intakeRoller.setIntakeRollerPercentOutput(output));
    }
}
