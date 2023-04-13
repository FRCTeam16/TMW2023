package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.pose.PoseManager;
import frc.robot.commands.pose.PoseManager.Pose;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake;


public class DoubleScore extends SequentialCommandGroup {
    
    public DoubleScore() {

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );

        ScoreHighHelper.scoreHighCone(this);

        addCommands(
            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),

            new ProfiledDistanceDriveCommand(180, 1, 0.5, 0)
                     .withEndSpeed(0.5)
                     .withThreshold(0.1)
                     .withTimeout(5.0),

            //here we need to turn
            new ProfiledDistanceDriveCommand(0, 0.5, 0.25, 0)
                     .withEndSpeed(0.5)
                     .withThreshold(0.1)
                     .withTimeout(0.3),

            Commands.parallel(
                new ProfiledDistanceDriveCommand(0, 1, 3, 0)
                     .withEndSpeed(0.5)
                     .withThreshold(0.1)
                     .withTimeout(5.0),

                new SchedulePose(Pose.SingleSubstation),
                new InstantCommand(Subsystems.intake::OpenHand),
                new InstantCommand(Subsystems.intake::intake),
                new WaitCommand(0.5)
            ),
            new SchedulePose(Pose.GroundPickup),
            new WaitCommand(0.5),

            new ProfiledDistanceDriveCommand(0, 1, 0.2, 0)
                     .withEndSpeed(0.5)
                     .withThreshold(0.1)
                     .withTimeout(5.0)


            

            // new ProfiledDistanceDriveCommand(180, 0.75, 0, DIR * 2)
            //     .withEndSpeed(0.5)
            //     .withTimeout(2.0)
        );
    }
}
