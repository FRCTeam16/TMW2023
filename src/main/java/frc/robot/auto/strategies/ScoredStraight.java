package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoredStraight extends SequentialCommandGroup {
    public ScoredStraight() {

        int DIR = (DriverStation.getAlliance() == Alliance.Red) ? 1 : -1;

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );
        
        ScoreHighHelper.scoreHighCone(this);

        addCommands(
            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),

            new ProfiledDistanceDriveCommand(180, 0.5, 4, 0)
        );
    }
}
