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

public class ScoreConeThenCube extends SequentialCommandGroup {

    public ScoreConeThenCube() {

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand),

            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.5),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint),

            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),

            // Drive out of community
            new ProfiledDistanceDriveCommand(180, 1, 1.9, 0)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(3.0),

            // Pose to ground pickup
            Commands.parallel(
                new SchedulePose(Pose.GroundPickup),
                Commands.run(Subsystems.intake::intake),
                Commands.run(Subsystems.intake::OpenHand),
                new WaitCommand(0.5)
            ).withTimeout(0.5),

            // Drive and pickup cube
            new ProfiledDistanceDriveCommand(15, 0.3, 2.25, 0.35)
                .withEndSpeed(0.0)
                .withThreshold(0.1)
                .withTimeout(3.0),
            
            new PrintCommand("Finished pickup"),

            // Pose to stow
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                Commands.run(Subsystems.intake::stopIntake),
                new WaitCommand(0.3)
            ).withTimeout(0.3),

            // Spin to face targets
            new ProfiledDistanceDriveCommand(180, 0.2, -0.5, -0.0)
                .withEndSpeed(0.2)
                .withTimeout(5.0),

            // Drive towards targets
            new ProfiledDistanceDriveCommand(180, 1, -2.0, -0.1)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(5.0),

            // Drive to front of target
            new SchedulePose(Pose.ScoreHighCone),
            new ProfiledDistanceDriveCommand(180, 0.6, -2.0, 0.5)
                .withEndSpeed(0.0)
                .withThreshold(0.1)
                .withTimeout(2.0),

            // Score
            Commands.run(Subsystems.intake::eject).withTimeout(0.25),
            new SchedulePose(Pose.Stow)
        );
    }
    
}
