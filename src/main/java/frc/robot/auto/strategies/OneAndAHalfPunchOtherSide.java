package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.OverTheRainbow.PitchDropWatcher;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.XWHeelLock;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.auto.StopDrive;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.Intake;
import frc.robot.commands.Balance;

public class OneAndAHalfPunchOtherSide extends SequentialCommandGroup {

    private PitchDropWatcher pitchWatcher = new PitchDropWatcher();


    public OneAndAHalfPunchOtherSide() {

        int DIR = (DriverStation.getAlliance() == Alliance.Red) ? 1 : -1;

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );
        ScoreHighHelper.scoreHighCone(this);

        addCommands(
            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.PreGroundPickup),

            // Drive out of community
            new ProfiledDistanceDriveCommand(180 * DIR, 1, 2.4, -0.26 * DIR)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(3.0),

            // Pose to ground pickup
            Commands.parallel(
                new SchedulePose(Pose.PreGroundPickup),
                Commands.run(Subsystems.intake::intake),
                new WaitCommand(0.5)
            ).withTimeout(0.5),

            new RotateToAngle(-7.4  * DIR).withThreshold(1).withTimeout(1),
            Commands.parallel(
                new StopDrive(),
                new SchedulePose(Pose.GroundPickup),
                new WaitCommand(0.25)
            ).withTimeout(0.25),

            // Drive and pickup cube
            new PrintCommand("Starting pickup"),
            Commands.parallel(
                new ProfiledDistanceDriveCommand(-7.53 * DIR, 0.25, 1.75, 0 * DIR)
                    .withEndSpeed(0.25)
                    .withTimeout(2)
                    .andThen(new StopDrive()),
                new ClampHandOnPart()
            ).withTimeout(2),
            new PrintCommand("Finished pickup"),

            // Pose to stow
            Commands.print("Stowing"),
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                new RotateToAngle(-178  * DIR).withThreshold(10).withTimeout(1.5)
            ).withTimeout(1.5),

            Commands.print("Moving Right"),
            new ProfiledDistanceDriveCommand(-178 * DIR, 0.4, -0.78, -1.5 * DIR)
                .withEndSpeed(0.4)
                .withThreshold(0.1)
                .withTimeout(1.5),

            new PrintCommand("Move Right"),

            new ProfiledDistanceDriveCommand(180 * DIR, 0.4, -1.8, 0 * DIR)
                .withEndSpeed(0.4)
                .withThreshold(0.1)
                .withTimeout(2),
            new PrintCommand("Finished moving to ramp"),

            new ProfiledDistanceDriveCommand(180 * DIR, 0.25, -1, 0 * DIR)
                .withStopCondition(this.pitchWatcher::shouldStopNoMaxWatch)
                .withThreshold(0.1)
                .withTimeout(2),

                // right before crawl, adjust shooter and punch
            new SchedulePose(Pose.Shoot),
            new WaitCommand(0.5),
            
            Commands.race(
                new InstantCommand(Subsystems.intake::OpenPuncher),
                new WaitCommand(0.035)
            ),

            new SchedulePose(Pose.Stow),
            new Balance(0.5)

        );
    }
    
}
