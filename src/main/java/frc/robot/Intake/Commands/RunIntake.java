package frc.robot.Intake.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeConfig;

public class RunIntake extends Command{
    public enum Mode {
        kIntake(IntakeConfig.kIntake), kHandOff(IntakeConfig.kHandoff);

        Mode(IntakeConfig.IntakeProfile profile) {
            kProfile = profile;
        }

        private final IntakeConfig.IntakeProfile kProfile;
    }

    private final Intake roller;
    private final Supplier<Mode> modeSupplier;
    private Mode lastMode;

    public RunIntake(Intake roller, Supplier<Mode> modeSupplier) {
        this.roller = roller;
        this.modeSupplier = modeSupplier;

        addRequirements(roller);
    }

    public RunIntake(Intake roller, Mode mode) {
        this.roller = roller;
        this.modeSupplier = () -> mode;

        addRequirements(roller);
    }

    @Override
    public String getName() {
        return "RunIntake";
    }

    @Override
    public void initialize() {
        lastMode = modeSupplier.get();
        roller.setProfile(lastMode.kProfile);
        roller.setIntakeRollerPercentOutput(1);
    }

    @Override
    public void execute() {
        final var currentMode = modeSupplier.get();
        if (currentMode != lastMode) {
            System.out.println("switching mode " + currentMode.name());
            lastMode = currentMode;
            roller.setProfile(lastMode.kProfile);
        }
    }

    @Override
    public void end(boolean interrupted) {
        roller.stop();
    }
}
