package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager;
import frc.robot.commands.pose.PoseManager.Pose;

public class DoubleScore extends SequentialCommandGroup {
    public DoubleScore(){

        addCommands(
            new InitializeAutoState(180),

            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.5),
            new InstantCommand(Subsystems.intake::OpenHand),

            Commands.parallel(

            new ProfiledDistanceDriveCommand(180, 0.25, 0, 0.5)
                .withEndSpeed(0.125)
                .withTimeout(3),
            new SchedulePose(Pose.Stow)

            ),

            new ProfiledDistanceDriveCommand(0, 0.75, 0, 2)
                .withEndSpeed(0.3)
            
        );
    }
}
