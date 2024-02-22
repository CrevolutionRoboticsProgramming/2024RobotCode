package frc.robot.Shooter.Commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.IntakeConfig;
import frc.robot.Intake.IntakePivot;
import frc.robot.Shooter.ShooterConfig;
import frc.robot.Shooter.ShooterPivot;

public class HoldPivot extends Command{
    private final ShooterPivot pivot;
    private ShooterConfig.PivotState state;

    public HoldPivot(ShooterPivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        state = pivot.getState();
        switch (state) {
            case kDeployed:
                pivot.setCurrentLimit(
                        IntakeConfig.kTensionContinuousCurrentLimit,
                        IntakeConfig.kTensionPeakCurrentLimit
                );
                pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;
            case kStowed:
                pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;
            default:
                pivot.setIdleMode(CANSparkMax.IdleMode.kBrake);
                break;
        }
        System.out.println("HoldState[" + state.name() + "]");
    }

    @Override
    public void execute() {
        if (state == ShooterConfig.PivotState.kDeployed) {
            pivot.setOutput(pivot.getAngleRads() < 0 ? IntakeConfig.kTensionOutput : IntakeConfig.kTensionFindOutput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        pivot.setCurrentLimit(IntakeConfig.kDefaultContinuousCurrentLimit, IntakeConfig.kDefaultPeakCurrentLimit);
        pivot.stop();
    }
}
