package frc.robot.commands.pose;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

class MoveToGroundPickupPose extends SequentialCommandGroup {
    public MoveToGroundPickupPose() {

        addCommands(
            new PoseElevator(ElevatorPosition.GroundPickup),
            new WaitCommand(0.3),
            Commands.parallel(
                (DriverStation.isTeleop() && !Subsystems.intake.hasPart()) ? new InstantCommand(Subsystems.intake::OpenHand) : new InstantCommand(), 
                new PosePivot(PivotPosition.GroundPickup),
                new PoseWrist(WristPosition.GroundPickup)
            )
        );
    }
}
