package frc.robot.auton;

import java.security.AuthProvider;

import com.fasterxml.jackson.databind.util.Named;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.commands.TurnInPlaceCommand;
import frc.robot.commands.RobotCommands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.intakeroller.commands.IntakeRollerCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterpivot.commands.SetAngleShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;

/* MASTER AUTON CLASS */
public class AutonMaster {
    private static Field2d mGameField;
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    private final Drivetrain drivetrain;

    public AutonMaster() {
        drivetrain = Drivetrain.getInstance();
        /* Define Named Commands Here */
        configureNamedCommands();

        //Configuring AutoBuilder
        AutoBuilder.configureHolonomic(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::getRobotRelativeSpeeds,
            drivetrain::driveRobotRelative,
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
            drivetrain
        );
 
        /* Add all auton options here */
        autonChooser.addOption("center-speaker-3p-right [verified]", AutoBuilder.buildAuto("center-speaker-3-right"));
        autonChooser.addOption("center-speaker-3p-left", AutoBuilder.buildAuto("center-speaker-3-left"));
        autonChooser.addOption("center-speaker-2p", AutoBuilder.buildAuto("center-speaker-2"));
        autonChooser.addOption("right-speaker-2.5p [verified]", AutoBuilder.buildAuto("right-speaker-2.5"));
        autonChooser.addOption("right-speaker-2p", AutoBuilder.buildAuto("right-speaker-2"));
        autonChooser.addOption("right-speaker-1-temp", AutoBuilder.buildAuto("right-speaker-1-temp"));
        autonChooser.addOption("left-speaker-2.5p", AutoBuilder.buildAuto("right-speaker-2.5"));
        autonChooser.addOption("left-speaker-2p", AutoBuilder.buildAuto("right-speaker-2"));
        autonChooser.addOption("TEST-center-speaker-3p-right", AutoBuilder.buildAuto("center-speaker-3-right-NEW"));
        autonChooser.addOption("center-speaker-3.5-neutral", AutoBuilder.buildAuto("center-speaker-3.5-neutral"));
        autonChooser.addOption("TEST-center-speaker-4.5p-under-stage", AutoBuilder.buildAuto("CenterSpeaker4.5PieceAuton"));
        autonChooser.addOption("TEST-center-speaker-4p", AutoBuilder.buildAuto("CenterSpeaker4PieceAuton"));
        autonChooser.addOption("TEST-feed-4p-auton", AutoBuilder.buildAuto("Feed4PieceAuton"));
        autonChooser.addOption("left-speaker-1.5p", AutoBuilder.buildAuto("left-speaker-1.5"));
        autonChooser.addOption("center-speaker-4-neutral", AutoBuilder.buildAuto("center-speaker-4-neutral"));
        autonChooser.addOption("Coyle's Stupid Auton", AutoBuilder.buildAuto("coyles-stupid-auton"));

        configurePathPlannerLogging();
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("ZeroHeading", new InstantCommand(drivetrain::zeroHeading));
        NamedCommands.registerCommand("autoPrimeSpeakerAndShootNear", RobotCommands.autoPrimeSpeakerAndShoot(
            SetAngleShooterPivot.Preset.kShooterNear,
            ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.8
        ));
        NamedCommands.registerCommand("autoPrimeSpeakerAndShootMid", RobotCommands.autoPrimeSpeakerAndShoot(
            SetAngleShooterPivot.Preset.kShooterMid,
            ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.8
        ));
        NamedCommands.registerCommand("autoPrimeSpeakerAndShootFar", RobotCommands.autoPrimeSpeakerAndShoot(
            SetAngleShooterPivot.Preset.kShooterFarAuton,
            ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.8
        ));
        NamedCommands.registerCommand("autoPrimeAndPass", RobotCommands.autoPrimeSpeakerAndShoot(
            SetAngleShooterPivot.Preset.kClimb,
            ShooterFlywheel.Settings.kMaxAngularVelocity.getRotations() * 0.25
        ));
        NamedCommands.registerCommand("autoRunIntake", Commands.parallel(
            IntakeRollerCommands.setOutput(() -> -1.0),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff)
        ));

        //TESTING (post-vision) Auton - Satchit
        NamedCommands.registerCommand("PerpetualRPM", RobotCommands.autoConstantlyRPM());
        NamedCommands.registerCommand("AutoLineupShoot", RobotCommands.autoLineupAndShoot());   
        NamedCommands.registerCommand("AutoHandOff", RobotCommands.autoHandOffNote());
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