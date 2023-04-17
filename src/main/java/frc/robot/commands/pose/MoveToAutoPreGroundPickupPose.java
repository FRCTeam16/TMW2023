package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToAutoPreGroundPickupPose extends SequentialCommandGroup {
    public MoveToAutoPreGroundPickupPose() {
        addCommands(
            new PoseElevator(ElevatorPosition.GroundPickup),
            new WaitCommand(0.3),
            Commands.parallel(
                new PosePivot(PivotPosition.AutoPreGround),
                new PoseWrist(
                    (Subsystems.partIndicator.isCubeRequested()) ? 
                        WristPosition.GroundPickupCube :
                        WristPosition.GroundPickup)
            )
        );
    }
}
