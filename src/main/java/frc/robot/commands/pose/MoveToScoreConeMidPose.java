package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToScoreConeMidPose extends SequentialCommandGroup {
    public MoveToScoreConeMidPose() {
        addCommands(
            Commands.parallel(
                new PosePivot(PivotPosition.ScoringAngle),
                new PoseElevator(ElevatorPosition.ScoreConeMid)
            ),
            new WaitCommand(0.20),
            Commands.parallel(
                Subsystems.intake.isHandOpen() ? 
                    new PoseWrist(WristPosition.ScoreCubeMid) :
                    new PoseWrist(WristPosition.ScoreCone)
            )
        );
    }
}
