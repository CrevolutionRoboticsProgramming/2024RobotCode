package frc.robot.Intake.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConfig;

public class IntakeCommands extends Command {
    @Deprecated
    public static Command setPivotState(IntakeConfig.PivotState state) {
        return new InstantCommand(() -> RobotContainer.intakePivot.setState(state), RobotContainer.intakePivot);
    }

    public static Command runRollerManual(Intake roller, double supplier) {
        return new InstantCommand(() -> roller.setIntakeRollerPercentOutput(supplier));
    }

    @Deprecated
    public static Command setPivotOutput(DoubleSupplier supplier) {
        return new InstantCommand(() -> RobotContainer.intakePivot.setOutput(supplier.getAsDouble()), RobotContainer.intakePivot);
    }


}
