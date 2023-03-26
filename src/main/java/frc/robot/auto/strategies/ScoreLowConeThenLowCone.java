package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.OverTheRainbow.PitchDropWatcher;
import frc.robot.commands.Balance;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.XWHeelLock;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoreLowConeThenLowCone extends SequentialCommandGroup {

    boolean coneMode = true;
    PitchDropWatcher pitchWatcher = new PitchDropWatcher(-13.0);

    public ScoreLowConeThenLowCone() {
        double pickupSpeed = coneMode ? 0.5 : 0.3;
        double pickupTimeout = coneMode ? 2.5 : 3.5;


        addCommands(
            new InitializeAutoState(0),
            Commands.parallel(
                new SchedulePose(Pose.GroundPickup),
                new WaitCommand(1.5),
                new InstantCommand(Subsystems.intake::OpenHand),
                new InstantCommand(Subsystems.intake::intake)
            ).withTimeout(1.5),

            // Drive out of community
            new ProfiledDistanceDriveCommand(0, 1, 3, 0)
                .withEndSpeed(pickupSpeed)
                .withThreshold(0.1)
                .withTimeout(3.0),


            // Drive and pickup cube
            Commands.parallel(
                new ProfiledDistanceDriveCommand(0, pickupSpeed, 1.5, 0)
                    .withEndSpeed(pickupSpeed)
                    .withThreshold(0.1)
                    .withTimeout(pickupTimeout),
                new ClampHandOnPart().withTimeout(pickupTimeout)
            ).withTimeout(pickupTimeout),
            
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
            (coneMode) ? Commands.print("cone mode skip pose") : new SchedulePose(Pose.ScoreHighCone),
            new ProfiledDistanceDriveCommand(180, 1, -2.5, -0.1)
                .withEndSpeed(0.6)
                .withThreshold(0.1)
                .withTimeout(5.0),

            // Drive to front of target
            new ProfiledDistanceDriveCommand(180, 0.6, -2.15, 0.55)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(2.0),

            // Score
            Commands.run(Subsystems.intake::eject).withTimeout(0.25),
            
            new ProfiledDistanceDriveCommand(180, 1, 1.75, 1.75)
                .withTimeout(3),
            // new Balance(),
            new ProfiledDistanceDriveCommand(180, 0.32, 2, 0)
                .withEndSpeed(0.32)
                .withStopCondition(this.pitchWatcher::shouldStopNoMaxWatch)
                .withTimeout(5.0),
            new XWHeelLock()
        );
    }
    
}
