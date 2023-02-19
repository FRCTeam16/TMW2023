package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class PoseElevator extends CommandBase {

  private final ElevatorPosition position;

  public PoseElevator(Elevator.ElevatorPosition position) {
    this.position = position;
  }

  @Override
  public void execute() {
    Subsystems.elevator.setElevatorPosition(position);
  }

  @Override
  public boolean isFinished() {
      return true;
  }
}
