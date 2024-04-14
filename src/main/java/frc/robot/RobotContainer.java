// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auton.AutonMaster;
import frc.robot.auton.commands.TurnInPlaceCommand;
import frc.robot.driver.Driver;
import frc.robot.driver.DriverXbox;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.commands.DrivetrainCommands;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorCommands;
import frc.robot.indexer.Indexer;
import frc.robot.intakepivot.IntakePivot;
import frc.robot.intakepivot.commands.IntakePivotCommands;
import frc.robot.shooterflywheel.ShooterFlywheel;
import frc.robot.shooterflywheel.commands.ShooterFlywheelCommands;
import frc.robot.shooterpivot.ShooterPivot;
import frc.robot.shooterpivot.commands.ShooterPivotCommands;
import frc.robot.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    AutonMaster mAutonMaster = new AutonMaster();

    // Gamepads


    /* Subsystems */

    /* Auton Chooser */
    public static SendableChooser<Command> mAutonChooser;

    public RobotContainer() {
        mAutonChooser = mAutonMaster.getAutonSelector();
        setDefaultCommands();

        ShuffleboardTab autonTab = Shuffleboard.getTab("Auton Chooser");
        autonTab.add(mAutonChooser);
        SmartDashboard.putData(mAutonChooser);

        ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
        Vision.PoseEstimator.getInstance().addDashboardWidgets(visionTab);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return mAutonChooser.getSelected();
    }

    public void setDefaultCommands() {
        // final var driver = Driver.getInstance();
        final var driver = DriverXbox.getInstance();
        Drivetrain.getInstance().setDefaultCommand(DrivetrainCommands.drive(
            driver::getDriveTranslation,
            driver::getDriveRotation
        ));        

        ShooterPivot.getInstance().setDefaultCommand(ShooterPivotCommands.setAngularVelocity(() -> Rotation2d.fromRotations(0), true));
        IntakePivot.getInstance().setDefaultCommand(IntakePivotCommands.setAngularVelocity(() -> Rotation2d.fromRotations(0), false));
    }
}
  