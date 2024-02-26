package frc.robot.drivetrain.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.crevolib.io.JoystickConfig;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public class TeleopDrive extends Command {
    private Drivetrain swerveDrivetrain;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private boolean isFieldRelative;

    public TeleopDrive(Drivetrain swerveDrivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, boolean isFieldRelative) {
        this.swerveDrivetrain = swerveDrivetrain;
        addRequirements(swerveDrivetrain);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.isFieldRelative = isFieldRelative;
    }


    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), JoystickConfig.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), JoystickConfig.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), JoystickConfig.stickDeadband);

        /* Drive */
        swerveDrivetrain.drive(
            new Translation2d(translationVal, strafeVal).times(DriveConstants.maxSpeed), 
            rotationVal * DriveConstants.maxAngularVelocity, 
            isFieldRelative, 
            true
        );
    }
}
