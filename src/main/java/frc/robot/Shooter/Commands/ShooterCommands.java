package frc.robot.Shooter.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterConfig;

public class ShooterCommands {
    @Deprecated
    public static Command setPivotState(ShooterConfig.PivotState state) {
        return new InstantCommand(() -> RobotContainer.shooterPivot.setState(state), RobotContainer.intakePivot);
    }

    public static Command runRollerManual(Shooter roller, double supplier) {
        return new InstantCommand(() -> roller.setShooterPercentOutput(supplier));
    }

    @Deprecated
    public static Command setPivotOutput(DoubleSupplier supplier) {
        return new InstantCommand(() -> RobotContainer.shooterPivot.setOutput(supplier.getAsDouble()), RobotContainer.intakePivot);
    }
}
