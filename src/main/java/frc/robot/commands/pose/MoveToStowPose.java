package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToStowPose extends SequentialCommandGroup {
    public MoveToStowPose() {
        addCommands(
            Commands.race(
                new PosePivot(PivotPosition.Stow),
                new WaitCommand(0.5)
            ),
            Commands.parallel(
                new PoseElevator(ElevatorPosition.Stow),
                new PoseWrist(WristPosition.Stow)
            )
        );
    }
    
}
