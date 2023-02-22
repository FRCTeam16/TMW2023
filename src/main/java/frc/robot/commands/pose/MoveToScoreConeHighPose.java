package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

public class MoveToScoreConeHighPose extends SequentialCommandGroup {
    public MoveToScoreConeHighPose() {
        addCommands(
            Commands.race(
                new PosePivot(PivotPosition.ScoringAngle),
                new WaitCommand(0.5)
            ),
            Commands.parallel(
                new PoseElevator(ElevatorPosition.ScoreConeHigh),
                new PoseWrist(WristPosition.ScoreCone)
            )
        );
    }
}
