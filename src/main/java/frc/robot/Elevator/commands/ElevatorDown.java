package frc.robot.Elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Elevator.Elevator;
import frc.robot.Elevator.ElevatorConfig;

public class ElevatorDown extends Command {
  
  private Elevator m_elevator;

  public ElevatorDown(Elevator elevator) {

    m_elevator = elevator;

    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {

    m_elevator.setElevatorOutput(ElevatorConfig.kMaxVelocityDown);

  }

  @Override
  public boolean isFinished() {

      return true;

  }
  
}

