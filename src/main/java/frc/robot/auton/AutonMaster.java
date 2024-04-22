package frc.robot.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.RobotCommands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.intakepivot.commands.SetStateIntakePivot;
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
        // autonChooser.addOption("center-speaker-3p-right [verified]", AutoBuilder.buildAuto("center-speaker-3-right"));
        // autonChooser.addOption("center-speaker-3p-left", AutoBuilder.buildAuto("center-speaker-3-left"));
        // autonChooser.addOption("center-speaker-2p", AutoBuilder.buildAuto("center-speaker-2"));
        // autonChooser.addOption("right-speaker-2.5p [verified]", AutoBuilder.buildAuto("right-speaker-2.5"));
        // autonChooser.addOption("right-speaker-2p", AutoBuilder.buildAuto("right-speaker-2"));
        // autonChooser.addOption("right-speaker-1-temp", AutoBuilder.buildAuto("right-speaker-1-temp"));
        // autonChooser.addOption("left-speaker-2.5p", AutoBuilder.buildAuto("right-speaker-2.5"));
        // autonChooser.addOption("left-speaker-2p", AutoBuilder.buildAuto("right-speaker-2"));
        // autonChooser.addOption("TEST-center-speaker-3p-right", AutoBuilder.buildAuto("center-speaker-3-right-NEW"));
        // autonChooser.addOption("center-speaker-3.5-neutral", AutoBuilder.buildAuto("center-speaker-3.5-neutral"));
        // // autonChooser.addOption("TEST-center-speaker-4.5p-under-stage", AutoBuilder.buildAuto("CenterSpeaker4.5PieceAuton"));
        // autonChooser.addOption("TEST-center-speaker-4p", AutoBuilder.buildAuto("CenterSpeaker4PieceAuton"));
        // // autonChooser.addOption("TEST-feed-4p-auton", AutoBuilder.buildAuto("Feed4PieceAuton"));
        // autonChooser.addOption("left-speaker-1.5p", AutoBuilder.buildAuto("left-speaker-1.5"));
        // autonChooser.addOption("center-speaker-4-neutral", AutoBuilder.buildAuto("center-speaker-4-neutral"));
        // autonChooser.addOption("Coyle's Stupid Auton", AutoBuilder.buildAuto("coyles-stupid-auton"));++
        

        //SATCHIT AUTOS PRE TROY PLEASE TEST ASAP. DO NOT RUN WITHOUT TESTING !!!!!
        // autonChooser.addOption("OPTIMIZED-PRE-TROY-center-speaker-4p", AutoBuilder.buildAuto("OPTIMIZED-center-speaker-4-piece"));
        //autonChooser.addOption("OPTIMIZED-PRE-TROY-center-speaker-4p", AutoBuilder.buildAuto("OPTIMIZED-center-speaker-non3175-4-piece"));
        // autonChooser.addOption("OPTIMIZED-PRE-TROY-center-speaker-4.5p", AutoBuilder.buildAuto("OPTIMIZED-center-speaker-4.5-piece"));
        // autonChooser.addOption("OPTIMIZED-PRE-TROY-left-speaker-steal-center-fun", AutoBuilder.buildAuto("OPTIMIZED-left-speaker-steal-center-fun"));
        // autonChooser.addOption("OPTIMIZED-PRE-TROY-center-speaker-middle-4-piece", AutoBuilder.buildAuto("OPTIMIZED-center-speaker-middle-4-piece"));
        // autonChooser.addOption("OPTIMZIED-PRE-TROY-right-speaker-middle-4-piece", AutoBuilder.buildAuto("OPTIMIZED-right-speaker-middle-4-piece"));
        
        ////////////////////////////////////////////////////////
        //////////////////// TROY 1 AUTONS ////////////////////
        ///////////////////////////////////////////////////////
        
        //ALL THE 4 piece or steal autons
        // autonChooser.setDefaultOption("FINALIZED-center-speaker-close-4p", AutoBuilder.buildAuto("FINALIZED-center-speaker-4-piece"));
        // autonChooser.addOption("FINALIZED-center-speaker-steal-4p", AutoBuilder.buildAuto("FINALIZED-center-speaker-steal-4-piece"));
        // autonChooser.addOption("FINALIZED-left-speaker-steal-3p", AutoBuilder.buildAuto("FINALIZED-left-speaker-steal-3-piece"));
        // autonChooser.addOption("FINALIZED-right-speaker-steal-4p", AutoBuilder.buildAuto("FINALIZED-right-speaker-steal-4-piece"));
        
        // //SAFETY: all the 3 piece autons
        // autonChooser.addOption("FINALIZED-center-speaker-3p-left", AutoBuilder.buildAuto("FINALIZED-center-speaker-3-piece-left"));
        // autonChooser.addOption("FINALIZED-center-speaker-3p-right", AutoBuilder.buildAuto("FINALIZED-center-speaker-3-piece-right"));


        /////////////////////////////////////////////////////
        //////////////// NEW AUTONS TO TEST ////////////////0

        ///////////////////////////////////////////////////
        autonChooser.setDefaultOption("NEW-center-speaker-close-4p", AutoBuilder.buildAuto("New-center-speaker-4-piece"));
        autonChooser.addOption("NEW-center-speaker-5-piece", AutoBuilder.buildAuto("NEW-center-speaker-5-piece"));
        autonChooser.addOption("NEW-center-speaker-close-4p-skip-AMP", AutoBuilder.buildAuto("NEW-center-speaker-4-piece-SkipAmp"));

        autonChooser.addOption("NEW-feed-side-speaker-4.5-piece", AutoBuilder.buildAuto("New-feed-speaker-4.5-piece"));
        autonChooser.addOption("NEW-feed-side-speaker-4.5-piece-skip-5", AutoBuilder.buildAuto("New-feed-speaker-4-piece-Skip5"));

        autonChooser.addOption("NEW-amp-side-speaker-4.5-piece", AutoBuilder.buildAuto("New-amp-speaker-4.5-piece"));
        autonChooser.addOption("NEW-amp-side-speaker-4-piece-Skip1", AutoBuilder.buildAuto("New-amp-speaker-4-piece-Skip1"));
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
            IntakePivotCommands.setPivotState(SetStateIntakePivot.State.kDeployed),
            IntakeRollerCommands.setOutput(() -> -1.0),
            ShooterPivotCommands.setState(SetAngleShooterPivot.Preset.kHandoff)
        ));
        
        NamedCommands.registerCommand("PerpetualRPM", RobotCommands.autoConstantlyRPM());
        NamedCommands.registerCommand("AutoLineupShoot", RobotCommands.autoLineupAndShoot());   
        NamedCommands.registerCommand("AutoHandOff", RobotCommands.autoHandOffNote());
        NamedCommands.registerCommand("IntakeDeployStowBeamBreak", RobotCommands.intakeStowBeamBreakControl());
        NamedCommands.registerCommand("AutoIntakeAndHandoff", RobotCommands.autoIntakeAndHandoffSimulatenously());
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