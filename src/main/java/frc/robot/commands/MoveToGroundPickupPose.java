package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;

public class MoveToGroundPickupPose extends SequentialCommandGroup {
    public MoveToGroundPickupPose() {
        addCommands(
           new InstantCommand(()   -> new PoseElevator(ElevatorPosition.GroundPickup) , Subsystems.elevator)
        );
    }
}
