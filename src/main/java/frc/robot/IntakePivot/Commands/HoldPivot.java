// package frc.robot.IntakePivot.Commands;

// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.IntakePivot.IntakePivot;
// import frc.robot.IntakePivot.IntakePivotConfig;
// import frc.robot.IntakeRoller.IntakeConfig;

// public class HoldPivot extends Command{
//     private final IntakePivot pivot;
//     private IntakePivotConfig.PivotState state;

//     public HoldPivot(IntakePivot pivot) {
//         this.pivot = pivot;
//         addRequirements(pivot);
//     }

//     @Override
//     public void initialize() {
//         state = pivot.getState();
//         switch (state) {
//             case kDeployed:
//                 pivot.setCurrentLimit(
//                         IntakePivotConfig.kDefaultContinuousCurrentLimit,
//                         IntakePivotConfig.kDefaultPeakCurrentLimit
//                 );
//                 pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
//                 break;
//             case kStowed:
//                 pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
//                 break;
//             default:
//                 pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
//                 break;
//         }
//         System.out.println("HoldState[" + state.name() + "]");
//     }

//     @Override
//     public void execute() {
//         if (state == IntakePivotConfig.PivotState.kDeployed) {
//             pivot.setOutput(pivot.getAngleRads() < 0 ? IntakePivotConfig.kTensionOutput : IntakePivotConfig.kTensionFindOutput);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         pivot.setCurrentLimit(IntakePivotConfig.kDefaultContinuousCurrentLimit, IntakePivotConfig.kDefaultPeakCurrentLimit);
//         pivot.stop();
//     }
// }
