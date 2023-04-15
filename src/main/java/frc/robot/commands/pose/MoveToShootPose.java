package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

public class MoveToShootPose extends SequentialCommandGroup{
    public MoveToShootPose() {
        addCommands(
            Commands.race(
                new PosePivot(PivotPosition.Shooter),
                new WaitCommand(0.5)
            ),
            Commands.parallel(
                new PoseElevator(ElevatorPosition.Shooter),
                new PoseWrist(WristPosition.Shooter)
            )
        );
    }
    
}
