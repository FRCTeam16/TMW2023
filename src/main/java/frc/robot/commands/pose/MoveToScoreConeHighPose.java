package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToScoreConeHighPose extends SequentialCommandGroup {
    public MoveToScoreConeHighPose() {
        if(Subsystems.pivot.getPivotEncoderPosition() > 30000 &&
           Subsystems.poseManager.getLastPose() != Pose.ScoreMidCone) {
            addCommands(
                new PosePivot(PivotPosition.Intermediate),
                new PoseElevator(ElevatorPosition.Intermediate),
                new WaitCommand(.25)
            );
        }
        addCommands(
            Commands.parallel(
                new PosePivot(PivotPosition.ScoringAngle),
                new PoseElevator(ElevatorPosition.ScoreConeHigh)
            ),
            new WaitCommand(0.25),
            Commands.parallel(
                Subsystems.intake.isHandOpen() ? 
                    new PoseWrist(WristPosition.ScoreCubeHigh) :
                    new PoseWrist(WristPosition.ScoreCone)
            )
        );
    }
}
