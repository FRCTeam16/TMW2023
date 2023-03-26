package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.HandState;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToSingleSubstationPose extends SequentialCommandGroup {
    public MoveToSingleSubstationPose() {
        addCommands(
            Commands.parallel(
                new PosePivot(PivotPosition.SingleSubstation),
                (Subsystems.intake.isProxTripped()) ? new InstantCommand() : new PoseHand(HandState.Open),
                new WaitCommand(0.5)
            ),
            Commands.parallel(
                new PoseElevator(ElevatorPosition.SingleSubstationCone),
                new PoseWrist(WristPosition.SingleSubstation)
            )
        );
    }
}
