package frc.robot.intakepivot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intakepivot.IntakePivotConfig;

public class IntakePivotCommands {
    public static Command setPivotState(SetStatePivot.State state) {
        return new SetStatePivot(state);
    }
    // public static Command setPivotOutput(DoubleSupplier supplier) {
    //     return new SetOutputPivot(supplier);
    // }
}
