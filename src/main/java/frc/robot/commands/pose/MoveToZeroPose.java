package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

public class MoveToZeroPose extends SequentialCommandGroup{
    public MoveToZeroPose() {
        addCommands(
            Commands.race(
                new PosePivot(PivotPosition.Vertical),
                new WaitCommand(0.5)
            ),
            Commands.parallel(
                new PoseElevator(ElevatorPosition.Down),
                new PoseWrist(WristPosition.Vertical)
            )
        );
    }
    
}
