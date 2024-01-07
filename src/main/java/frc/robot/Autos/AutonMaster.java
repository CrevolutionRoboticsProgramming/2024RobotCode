package frc.robot.Autos;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Drivetrain.DrivetrainConfig;

/* MASTER AUTON CLASS */
public class AutonMaster {
    /* Map is a (key, value) pair. The key is the name of the events performed in auton, value is the command associated */
    /* For ex. if you have a "ShootRingFar" event in auton, that would be the key, and value is the ShootRingFar command (if it is coded) */
    public static final Map<String, Command> eventMap = new HashMap<>(Map.ofEntries(
            Map.entry("StopDrivetrain", new InstantCommand(RobotContainer.mSwerveDrivetrain::stopSwerve)),
            Map.entry("ZeroGyro", new InstantCommand(() -> {
                RobotContainer.mSwerveDrivetrain.gyro.setYaw(RobotContainer.mSwerveDrivetrain.getGyroYaw().getDegrees());
            }))
    ));
    
    //TODO: need to update some values in here such as driveBaseRadius and maxModuleSpeed
    //TODO: No clue why new AutoBuilder does't build
    // AutoBuilder.configureHolonomic(
    //     RobotContainer.mSwerveDrivetrain::getPose, 
    //     RobotContainer.mSwerveDrivetrain::resetPose, 
    //     RobotContainer.mSwerveDrivetrain::getRobotRelativeSpeeds,
    //     RobotContainer.mSwerveDrivetrain::driveRobotRelative, 
    //     new HolonomicPathFollowerConfig(
    //         new PIDConstants(5,0,0), 
    //         new PIDConstants(5, 0, 0), 
    //         4.5, 
    //         0.4, 
    //         new ReplanningConfig()), 
    //         RobotContainer.mSwerveDrivetrain
    //     );

    /* Path Constraints Enum -> Add Depending on the Auton Modes we desire -> Slow, Medium, Fast Speeds */
    private enum AutonPathConstraints {
        //TODO: fix these values before running auton (espeically Angular values)
        kGeneric(4, 3, 3, 3),
        kSlow(1, 2, 1, 2);

        AutonPathConstraints(double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
            constraints = new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
        }

        PathConstraints get() { 
            return constraints;
        }

        private final PathConstraints constraints;
    }

    /* Auton Path Enum -> Add all the possible Auton Names/Path once Pathplanner update comes out */
    
}
