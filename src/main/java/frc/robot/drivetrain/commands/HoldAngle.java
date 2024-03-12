package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.DrivetrainConfig.DriveConstants;

public class HoldAngle extends Command{
    private final Drivetrain mDrivetrain;
    Rotation2d deltaTheta;
    private double distance;
    private Long startTs;
    private PIDController posPIDController, velPIDController;

    private static class Settings {
        //Positional PID Constatns
        static final double kPosP = 5.0;
        static final double kPosI = 0.0;
        static final double kPosD = 0.0;

        //Velocity PID Constatns
        static final double kVelP = 5.0;
        static final double kVelI = 0.0;
        static final double kVelD = 0.0;

        // Robot Radius (diagonal) in meters
        static final double radius = 0.7112 / 2.0; 
    }
    

    public HoldAngle(Rotation2d deltaTheta) {
        mDrivetrain = Drivetrain.getInstance();
        this.deltaTheta = deltaTheta;

        posPIDController = new PIDController(Settings.kPosP, Settings.kPosI, Settings.kPosD);
        velPIDController = new PIDController(Settings.kVelP, Settings.kVelP, Settings.kVelP);

        addRequirements(mDrivetrain);
    }

    @Override
    public void initialize() {
        distance = Settings.radius * deltaTheta.getRadians();
    }   

    @Override
    public void execute() {
        var pos = posPIDController.calculate(distance);

        var vel = velPIDController.calculate(pos);

        mDrivetrain.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(vel, new Rotation2d(Units.degreesToRadians(-45))),
            new SwerveModuleState(vel, new Rotation2d(Units.degreesToRadians(225))),
            new SwerveModuleState(vel, new Rotation2d(Units.degreesToRadians(45))),
            new SwerveModuleState(vel, new Rotation2d(Units.degreesToRadians(135)))
        });
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrain.stopSwerve();
    }
}
