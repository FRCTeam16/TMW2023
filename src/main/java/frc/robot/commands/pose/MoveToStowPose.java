package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToStowPose extends SequentialCommandGroup {
    public MoveToStowPose() {

        //Rotate first if in a down (scoring)

        if(Subsystems.pivot.getPivotEncoderPosition() > 100000) {
            addCommands(
            new PosePivot(PivotPosition.SingleSubstation),
            new WaitCommand(.25),
            new PoseElevator(ElevatorPosition.Stow),
            new WaitCommand(.25)
            );
        }

        addCommands(
            Commands.parallel(
                new PosePivot(PivotPosition.Stow),
                new PoseWrist(WristPosition.Stow)
            )
        );
    }
    
}
