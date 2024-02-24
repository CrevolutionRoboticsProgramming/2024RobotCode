package frc.robot.IntakePivot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.IntakePivot.IntakePivotConfig;
import frc.robot.IntakeRoller.IntakeConfig;

public class IntakePivotCommands {
    @Deprecated
    public static Command setPivotState(IntakePivotConfig.PivotState state) {
        return new InstantCommand(() -> RobotContainer.intakePivot.setState(state), RobotContainer.intakePivot);
    }

    @Deprecated
    public static Command setPivotOutput(DoubleSupplier supplier) {
        return new InstantCommand(() -> RobotContainer.intakePivot.setOutput(supplier.getAsDouble()), RobotContainer.intakePivot);
    }
}
