// package frc.robot.Shooter.Commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.IntakePivot.IntakePivot;
// import frc.robot.IntakeRoller.IntakeConfig;
// import frc.robot.Shooter.ShooterConfig;
// import frc.robot.Shooter.ShooterPivot;

// public class RunPivotManual extends Command{
//     private final ShooterPivot pivot;
//     private final DoubleSupplier supplier;

//     public RunPivotManual(ShooterPivot pivot, DoubleSupplier supplier) {
//         this.pivot = pivot;
//         this.supplier = supplier;

//         addRequirements(pivot);
//     }

//     @Override
//     public String getName() {
//         return getClass().getName();
//     }

//     @Override
//     public void initialize() {
//         pivot.setState(ShooterConfig.PivotState.kUnspecified);
//     }

//     @Override
//     public void execute() {
//         final var input = supplier.getAsDouble();
//         if (input < 0 && pivot.getAngleRads() < 0) {
//             pivot.stop();
//         } else {
//             pivot.setOutput(input);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         pivot.setState(pivot.getAngleRads() < 0 ? ShooterConfig.PivotState.kHandOff : ShooterConfig.PivotState.kUnspecified);
//         pivot.stop();
//     }
// }
