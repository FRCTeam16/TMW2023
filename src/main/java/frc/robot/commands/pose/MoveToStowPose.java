package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems; 
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToStowPose extends SequentialCommandGroup {
    public MoveToStowPose() {

        //Rotate first if in a down (scoring)
        if(Subsystems.pivot.getPivotEncoderPosition() > 100_000) {
            addCommands(
            new PosePivot(PivotPosition.Intermediate),
            new PoseElevator(ElevatorPosition.Intermediate),
            new WaitCommand(0.5)
            );
        }

        addCommands(
            new PoseElevator(ElevatorPosition.Stow),
            new WaitCommand(.25),
            new PosePivot(PivotPosition.Stow),
            new PoseWrist(WristPosition.Stow)
        );
    }
    
}
