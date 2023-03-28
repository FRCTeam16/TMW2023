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

public class DisableingCodeForTest extends SequentialCommandGroup {
    public DisableingCodeForTest(){

        addCommands(
            new InitializeAutoState(180),

            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.5),

            new InstantCommand(Subsystems.intake::OpenHand),
            new InstantCommand(Subsystems.intake::eject),

            new WaitCommand(5),
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                new ProfiledDistanceDriveCommand(180, 0.15, 0, 0.5)
            ),

            new WaitCommand(5),
            new SchedulePose(Pose.Zero),
            new WaitCommand(5)

            //Do comment this out for now bc untested, Do test though

            // new ProfiledDistanceDriveCommand(0, .05, 0, 0.25),
            // new WaitCommand(1),
            // new ProfiledDistanceDriveCommand(0, .05, 0, 1.5),
            // new WaitCommand(8),
            // new SchedulePose(Pose.SingleSubstation),
            // new WaitCommand(3),
            // Commands.parallel(
            //     new SchedulePose(Pose.GroundPickup),
            //     new InstantCommand(Subsystems.intake::intake)
            // ),
            // new WaitCommand(3),
            // Commands.parallel(
            // new ProfiledDistanceDriveCommand(0, 0.5, 0, .5),
            // new InstantCommand(Subsystems.intake::CloseHand)
            // )

            //another untested segment ( assume has cube )

            // new SchedulePose(Pose.SingleSubstation),
            // new WaitCommand(3),
            // new SchedulePose(Pose.Stow),
            // new WaitCommand(5),
            // new ProfiledDistanceDriveCommand(180, 0.15, 0, -0.5),
            // new WaitCommand(3),
            // new ProfiledDistanceDriveCommand(0, 0.05, 0, 0)
        );
    }
}
