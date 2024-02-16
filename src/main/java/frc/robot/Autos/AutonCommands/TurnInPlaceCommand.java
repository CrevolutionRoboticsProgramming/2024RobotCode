package frc.robot.Autos.AutonCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.Drivetrain;

public class TurnInPlaceCommand extends Command{
    private double angle;
    private Drivetrain mSwerveDrivetrain;

    public TurnInPlaceCommand(double angle, Drivetrain mSwerveDrivetrain) {
        this.angle = angle;
        this.mSwerveDrivetrain = mSwerveDrivetrain;

        addRequirements(mSwerveDrivetrain);
    }

    @Override
    public void execute() {
        //TODO: test if the angle is deg or radians
        mSwerveDrivetrain.drive(new Translation2d(0,0), Units.degreesToRadians(angle), true, true);
    }


    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
