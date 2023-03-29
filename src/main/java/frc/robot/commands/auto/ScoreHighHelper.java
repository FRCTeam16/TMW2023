package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoreHighHelper {
    public static void scoreHighCone(SequentialCommandGroup group) {
        group.addCommands(
            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.0),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint)
        );
    }
}
