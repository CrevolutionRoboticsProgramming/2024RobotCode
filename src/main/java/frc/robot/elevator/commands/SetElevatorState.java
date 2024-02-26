// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorConfig;
import frc.robot.elevator.ElevatorConfig.ElevatorState;

public class SetElevatorState extends Command {
  private final Elevator mElevator;
  private final ElevatorConfig.ElevatorState targetState;

  private TrapezoidProfile profile;
  private final ArmFeedforward ffController;
  private final PIDController pidController;

  private Long startTs;

   public SetElevatorState(Elevator elevator, ElevatorConfig.ElevatorState targetState) {
    mElevator = elevator;
    this.targetState = targetState;

    ffController = new ArmFeedforward(ElevatorConfig.kS, ElevatorConfig.kG, ElevatorConfig.kV, ElevatorConfig.kA);
    pidController = new PIDController(ElevatorConfig.kP, ElevatorConfig.kI, ElevatorConfig.kD);

    startTs = null;

    addRequirements(elevator);
  }

  @Override
  public String getName() {
    return "SetState[" + targetState.name() + "]";
  }

  @Override
  public void initialize() {

    profile = generateProfile();

    startTs = null;
  }

  @Override
  public void execute() {
    if (startTs == null) {
      startTs = System.currentTimeMillis();
    }

    final var targetState = profile.calculate(getElapsedTime());

    final var ffOutput = ffController.calculate(mElevator.getPositionMeters(), targetState.velocity);
    final var pidOutput = pidController.calculate(mElevator.getVelocityMps(), targetState.velocity);

    mElevator.setOutput((ffOutput + pidOutput) / 12.0);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(getName() + ": end");
        boolean[] elevatorLimitSwitchStates = mElevator.getLimitStates();
    
        if (elevatorLimitSwitchStates[0]) {
            mElevator.setState(ElevatorState.kZero);
        } else if (elevatorLimitSwitchStates[1]) {
            mElevator.setState(ElevatorState.kHigh);
        } else {
            mElevator.setState(interrupted ? ElevatorState.kUnspecified : targetState);
        }

        mElevator.stop();
  }

  @Override
  public boolean isFinished() {
    final var limitStates = mElevator.getLimitStates();
        final var time = getElapsedTime();
        if (limitStates[0] && profile != null && profile.calculate(time).velocity < 0 && time > 0.25) {
            System.out.println("[elevator] canceling due to low limit");
            return true;
        } 

        if (limitStates[1] && profile != null && profile.calculate(time).velocity > 0 && time > 0.25) {
            System.out.println("[elevator] canceling due to high limit");
            return true;
        }

        return (profile != null && profile.isFinished(time));
  }

  private double getElapsedTime() {
    return (startTs == null) ? 0 : ((double) System.currentTimeMillis() - startTs) / 1000.0;
  }

  private TrapezoidProfile generateProfile() {
    return new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConfig.kMaxVelocity,
                ElevatorConfig.kMaxAcceleration
            ),
            new TrapezoidProfile.State(targetState.target, 0),  //goal state
            new TrapezoidProfile.State(mElevator.getPositionMeters(), mElevator.getVelocityMps())); //start state
  } 
}