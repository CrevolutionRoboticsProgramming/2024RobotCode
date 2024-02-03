package frc.robot.Elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorConfig;

public class ElevatorTrap extends Command {
  private final Elevator elevator;
  private ElevatorConfig.ElevatorState targetState;


  public ElevatorTrap(Elevator elevator) {
    this.elevator = elevator;
    targetState = ElevatorConfig.ElevatorState.kUnspecified;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    targetState = elevator.getState();
                   
  }

  
  @Override
  public void execute() {
    final var limitStates = elevator.getLimitStates();
    elevator.setElevatorOutput(ElevatorConfig.kSeekVoltage);
    
    if (limitStates[0]) {
    elevator.setState(ElevatorConfig.ElevatorState.kTrap);
    targetState = ElevatorConfig.ElevatorState.kTrap;
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }


  @Override
  public boolean isFinished() {
    return true;
  }
}
