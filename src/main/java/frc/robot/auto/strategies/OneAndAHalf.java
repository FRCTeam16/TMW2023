package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

public class OneAndAHalf extends SequentialCommandGroup {

    private PitchDropWatcher pitchWatcher = new PitchDropWatcher();


    public OneAndAHalf() {
        this(false);
    }

    public OneAndAHalf(boolean invertSide) {

        int DIR = (DriverStation.getAlliance() == Alliance.Red) ? 1 : -1;

        if (invertSide) {
            System.out.println("Inverting Score And Balance direction");
            DIR *= -1;
        }

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
            new ProfiledDistanceDriveCommand(180 * DIR, 1, 1.9, 0 * DIR)
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

            new RotateToAngle(15 * DIR).withTimeout(1),
            Commands.parallel(
                new StopDrive(),
                new SchedulePose(Pose.GroundPickup),
                new WaitCommand(0.25)
            ),

            // Drive and pickup cube
            new PrintCommand("Starting pickup"),
            Commands.parallel(
                new ProfiledDistanceDriveCommand(15 * DIR, 0.5, 1.8, 0.021 * DIR)
                    .withEndSpeed(0.5)
                    .withThreshold(0.1)
                    .withTimeout(3)
                    .andThen(new StopDrive()),
                new ClampHandOnPart().withTimeout(3)
            ).withTimeout(3),
            // new InstantCommand(Subsystems.intake::CloseHand),
            new PrintCommand("Finished pickup"),

            // Pose to stow
            Commands.print("Stowing"),
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                new RotateToAngle(180 * DIR).withTimeout(1.5)
            ).withTimeout(1.5)
        );
    }
    
}
