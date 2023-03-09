package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

public class TestDrive extends SequentialCommandGroup {

    public TestDrive() {

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand),

            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.0),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint),

            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),

            new ProfiledDistanceDriveCommand(180, 1, 2, 0)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(3.0),

            // new ProfiledDistanceDriveCommand(0, 0.2, 0.5, 0)
            //     .withThreshold(0.2)
            //     .withEndSpeed(0.25)
            //     .withTimeout(1.0),

            Commands.parallel(
                new SchedulePose(Pose.SingleSubstation),
                Commands.run(Subsystems.intake::intake),
                Commands.run(Subsystems.intake::OpenHand)
            ).withTimeout(0.5),
            new SchedulePose(Pose.GroundPickup).withTimeout(0.5),

            new ProfiledDistanceDriveCommand(15, 0.3, 2.25, 0.35)
                .withEndSpeed(0.0)
                .withThreshold(0.1)
                .withTimeout(5.0),
            
            new PrintCommand("Finished pickup"),

            Commands.parallel(
                new SchedulePose(Pose.ScoreMidCone),
                Commands.run(Subsystems.intake::stopIntake)
            ).withTimeout(0.5),
            new SchedulePose(Pose.Stow).withTimeout(0.25),

            new ProfiledDistanceDriveCommand(180, 0.3, -0.5, -0.0)
                .withEndSpeed(0.3)
                .withTimeout(5.0),

            new ProfiledDistanceDriveCommand(180, 1, -2.0, -0.15)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(5.0),

            new SchedulePose(Pose.ScoreHighCone),
            new ProfiledDistanceDriveCommand(180, 0.6, -1.95, 0.45)
                .withEndSpeed(0.0)
                .withThreshold(0.1)
                .withTimeout(5.0),

            Commands.run(Subsystems.intake::eject).withTimeout(0.25),
            new SchedulePose(Pose.Stow)
        );
    }
    
}
