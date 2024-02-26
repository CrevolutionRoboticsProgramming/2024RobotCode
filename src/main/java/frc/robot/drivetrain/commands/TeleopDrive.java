package frc.robot.drivetrain.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.crevolib.io.JoystickConfig;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public class TeleopDrive extends Command {
    private final Drivetrain drivetrain;
    private final Supplier<Translation2d> translationSupplier;
    private final Supplier<Rotation2d> rotationSupplier;
    // TODO: replace w/ enum
    private final boolean isFieldRelative;

    /**
     *
     * @param translationSupplier translation demand, magnitude should be of interval [-1, 1], percent of max translational velocity
     * @param rotationSupplier interval from [-1, 1], percent of max angular velocity
     * @param isFieldRelative field relative or robot centric
     * @param rotationOffset offset for the robot's center of rotation
     */
    public TeleopDrive(Supplier<Translation2d> translationSupplier, Supplier<Rotation2d> rotationSupplier, boolean isFieldRelative,
                       Translation2d rotationOffset) {
        drivetrain = Drivetrain.getInstance();
        this.translationSupplier = translationSupplier;
        this.rotationSupplier = rotationSupplier;
        this.isFieldRelative = isFieldRelative;

        addRequirements(drivetrain);
    }


    @Override
    public void execute() {
        drivetrain.drive(
            translationSupplier.get(),
            rotationSupplier.get().getRadians(),
            isFieldRelative,
            true
        );
    }
}
