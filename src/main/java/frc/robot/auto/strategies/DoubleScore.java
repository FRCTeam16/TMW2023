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
            new InstantCommand(Subsystems.intake::eject),

            new WaitCommand(3),
            
            new SchedulePose(Pose.SingleSubstation),
            //there needs to be an intermdiary step here so it goes down correctly, else it will be considered ileagul and SPIN with the intake out
            new WaitCommand(3),
            Commands.parallel(
            new SchedulePose(Pose.Stow),
            new ProfiledDistanceDriveCommand(180, 0.1, 0, 0.5)
            ),

            new WaitCommand(5),
            new SchedulePose(Pose.Zero)

            
            
        );
    }
}
