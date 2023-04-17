package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.auto.StopDrive;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.PartIndicator.PartType;
import frc.robot.subsystems.vision.Pipeline;

public class TwoScoreOtherSide extends SequentialCommandGroup {

    public TwoScoreOtherSide() {

        int DIR = (DriverStation.getAlliance() == Alliance.Red) ? 1 : -1;

        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );
        ScoreHighHelper.scoreHighCone(this);

        addCommands(
            // new SchedulePose(Pose.SingleSubstation),
            // new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),
            new WaitCommand(0.5),

            // Drive out of community
            new ProfiledDistanceDriveCommand(180 * DIR, 1, 3.8, -0.25 * DIR)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(3.0),

            Commands.parallel(
                new StopDrive(),
                new InstantCommand(() -> Subsystems.intake.intake()),
                new InstantCommand(() -> Subsystems.intake.OpenHand())
            ),

            // Spin to face target
            new RotateToAngle(-7.5  * DIR)
                .withThreshold(5)
                .withTimeout(1.5),
            
            new SchedulePose(Pose.AutoPreGroundPickup),
            new VisionAlign()
                .withVisionPipeline(Pipeline.Cube)
                .withRobotAngle(-7.5 * DIR)
                .withTolerance(6)
                .withRobotSpeed(0.5)
                .withTimeout(1.5),

            new InstantCommand(() -> Subsystems.partIndicator.requestPart(PartType.Cube)),
            new SchedulePose(Pose.GroundPickup),
            new WaitCommand(0.5),

            Commands.parallel(
                new StopDrive(),
                new SchedulePose(Pose.GroundPickup),
                new WaitCommand(0.25)
            ).withTimeout(0.25),
            

            // Drive and pickup cube
            new PrintCommand("Starting pickup"),
            Commands.race(
                new ProfiledDistanceDriveCommand(-7.5 * DIR, 0.25, 0.7, 0 * DIR)
                    .withEndSpeed(0.25)
                    .withRobotCentric()
                    .withTimeout(2)
                    .andThen(new StopDrive()),
                new ClampHandOnPart(false)
            ).withTimeout(2),
            new PrintCommand("Finished pickup"),

            // Pose to stow
            Commands.print("Stowing"),
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                new RotateToAngle(-178  * DIR)
                    .withThreshold(5)
                    .withTimeout(1.5)
            ),

            new ProfiledDistanceDriveCommand(180 * DIR, 1, -4.0, -0.25 * DIR)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(3.0)
        );
    }
    
}
