package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoredStraight extends SequentialCommandGroup {
    public ScoredStraight() {
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand),
            new WaitCommand(0.5),
            
            Subsystems.poseManager.getPose(Pose.ScoreHighCone),
            new WaitCommand(1.0),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.5),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint),

            Subsystems.poseManager.getPose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            Subsystems.poseManager.getPose(Pose.Stow),

            new ProfiledDistanceDriveCommand(180, 0.5, 4, 0)
        );
    }
}
