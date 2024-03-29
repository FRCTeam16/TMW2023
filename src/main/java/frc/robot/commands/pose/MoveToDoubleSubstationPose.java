package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.Intake.HandState;

class MoveToDoubleSubstationPose extends SequentialCommandGroup {
    public MoveToDoubleSubstationPose() {

        addCommands(
                Commands.parallel(
                        new PosePivot(PivotPosition.DoubleSubstation),
                        (Subsystems.intake.isProxTripped()) ? new InstantCommand() : new PoseHand(HandState.Open),
                        new WaitCommand(0.5)),
                Commands.parallel(
                        new PoseElevator(ElevatorPosition.DoubleSubstation),
                        new PoseWrist(WristPosition.DoubleSubstation)));
    }

}
