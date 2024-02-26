package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.commands.TurnInPlaceCommand;

/* MASTER AUTON CLASS */
public class AutonMaster {
    private static Field2d mGameField;
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    public AutonMaster() {
        /* Define Named Commands Here */

        //Zero Heading -> use at the beginning and end of every auton
        NamedCommands.registerCommand("ZeroHeading", new InstantCommand(() -> {
                                RobotContainer.mSwerveDrivetrain.zeroHeading();}));
        
        NamedCommands.registerCommand("ResetFieldOrientation", new InstantCommand( () -> {
            RobotContainer.mSwerveDrivetrain.swerveOdometry.resetPosition(Rotation2d.fromDegrees(-180), 
                        RobotContainer.mSwerveDrivetrain.getModulePositions(), 
                new Pose2d(new Translation2d(1.89, 7.73), Rotation2d.fromDegrees(0)));   
        }));

        //Wait Command -> Common Command for Robot to Wait
        NamedCommands.registerCommand("WaitCommand", new WaitCommand(5));

        //Turn in place command -> enter custom angle 
        //This turns 45 deg.
        NamedCommands.registerCommand("TurnInPlace", new TurnInPlaceCommand(45, RobotContainer.mSwerveDrivetrain));

        //Configuring AutoBuilder
        AutoBuilder.configureHolonomic(
                RobotContainer.mSwerveDrivetrain::getPose, 
                RobotContainer.mSwerveDrivetrain::resetPose, 
                RobotContainer.mSwerveDrivetrain::getRobotRelativeSpeeds, 
                RobotContainer.mSwerveDrivetrain::driveRobotRelative, 
                new HolonomicPathFollowerConfig( 
                        AutonConfig.TRANSLATION_PID, 
                        AutonConfig.ROTATION_PID,
                        AutonConfig.MAX_AUTON_MODULE_SPEED, 
                        AutonConfig.DRIVE_BASE_RADIUS, 
                        new ReplanningConfig() 
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
            RobotContainer.mSwerveDrivetrain 
        );
       
        /* Add all auton options here */
        // autonChooser.setDefaultOption("CurveNoRotationAuto", AutoBuilder.buildAuto("CurveNoRotationAuto"));
        // autonChooser.addOption("CurveWithRotationAuto", AutoBuilder.buildAuto("CurveWithRotationAuto"));
        // autonChooser.addOption("2NoteAmpEndAuto", AutoBuilder.buildAuto("2NoteAmpEndAuto"));
        autonChooser.setDefaultOption("RightSpeaker2PieceAuton", AutoBuilder.buildAuto("RightSpeaker2PieceAuton"));
        autonChooser.addOption("CenterSpeaker3PieceAuton", AutoBuilder.buildAuto("CenterSpeaker3PieceAuton"));
        autonChooser.addOption("CenterSpeaker4PieceAuton", AutoBuilder.buildAuto("CenterSpeaker4PieceAuton"));
        configurePathPlannerLogging();
    }
    

    public SendableChooser<Command> getAutonSelector() {
        return autonChooser;
    }

    private void configurePathPlannerLogging() {
        mGameField = new Field2d();
        SmartDashboard.putData("Field", mGameField);
        
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            mGameField.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            mGameField.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            mGameField.getObject("path").setPoses(poses);
        });
    }

}
