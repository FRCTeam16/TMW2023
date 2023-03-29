package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoreConeThenCube extends SequentialCommandGroup {

    boolean coneMode = true;

    public ScoreConeThenCube() {
        double pickupSpeed = coneMode ? 0.5 : 0.3;
        double pickupTimeout = coneMode ? 2.5 : 3.5;
        // double pickupX = coneMode ? :;
        // double pickupY = coneMode ? :;

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );

        ScoreHighHelper.scoreHighCone(this);

        addCommands(

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
                new SchedulePose(Pose.PreGroundPickup),
                Commands.run(Subsystems.intake::intake),
                Commands.run(Subsystems.intake::OpenHand),
                new WaitCommand(0.5)
            ).withTimeout(0.5),

            new RotateToAngle(15).withTimeout(1),


            // Drive and pickup cube
            Commands.parallel(
                new SchedulePose(Pose.GroundPickup),
                new ProfiledDistanceDriveCommand(15, pickupSpeed, 1.8, 0.022)
                    .withEndSpeed(pickupSpeed)
                    .withThreshold(0.1)
                    .withTimeout(pickupTimeout),
                new ClampHandOnPart().withTimeout(pickupTimeout)
            ).withTimeout(pickupTimeout),
            
            new PrintCommand("Finished pickup"),

            // Pose to stow
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                // Commands.run(Subsystems.intake::stopIntake),
                new WaitCommand(0.3)
            ).withTimeout(0.3),

            // Spin to face targets
            new ProfiledDistanceDriveCommand(180, 0.2, -0.5, -0.0)
                .withEndSpeed(0.2)
                .withTimeout(5.0),

            // Drive towards targets
            (coneMode) ? Commands.print("cone mode skip pose") : new SchedulePose(Pose.ScoreHighCone),
            new ProfiledDistanceDriveCommand(180, 1, -2.0, -0.1)
                .withEndSpeed(0.6)
                .withThreshold(0.1)
                .withTimeout(5.0),

            // Drive to front of target
            new ProfiledDistanceDriveCommand(180, 0.6, -2.65, 0.35)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(2.0),

            // Score
            Commands.run(Subsystems.intake::eject).withTimeout(0.25),
            
            (coneMode) ? Commands.print("Skip stow pose") : new SchedulePose(Pose.Stow),
            (coneMode) ? Commands.print("Skip stop wait") : new WaitCommand(0.5)
        );
    }
    
}
