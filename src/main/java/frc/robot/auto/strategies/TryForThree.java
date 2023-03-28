package frc.robot.auto.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.pose.PoseManager.Pose;

public class TryForThree extends SequentialCommandGroup {

    boolean coneMode = true;
    PitchDropWatcher pitchWatcher = new PitchDropWatcher(-13.0);

    public TryForThree() {
        double pickupTimeout = 2.5;


        addCommands(
            new InitializeAutoState(0),
            new InstantCommand(() -> Subsystems.swerveSubsystem.hardResetOdometry(0)),
            new WaitCommand(.1),
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                new WaitCommand(1.5),
                new InstantCommand(Subsystems.intake::OpenHand),
                new InstantCommand(Subsystems.intake::intake)
            ).withTimeout(1.5),

            // Drive out of community
            new ProfiledDistanceDriveCommand(0, 0.4, 1.0, 0)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(1.25),

            // Drive and pickup cone
            Commands.parallel(
                new SchedulePose(Pose.GroundPickup),
                new ProfiledDistanceDriveCommand(0, 0.5, 0.6, 0)
                    .withEndSpeed(0.5)
                    .withThreshold(0.1)
                    .withTimeout(2.5),
                new ClampHandOnPart().withTimeout(2.5)
            ).withTimeout(2.5),
            new InstantCommand(Subsystems.intake::CloseHand),
            new PrintCommand("Finished first pickup"),

            // Pose to stow
            Commands.parallel(
                new SchedulePose(Pose.Stow),
                Commands.run(Subsystems.intake::stopIntake),
                new WaitCommand(0.3)
            ).withTimeout(0.3),

            // Spin to face goals
            new ProfiledDistanceDriveCommand(-170, 0.2, -0.5, -0.0)
                .withEndSpeed(0.2)
                .withTimeout(5.0),

            // Drive towards goals
            new ProfiledDistanceDriveCommand(180, 0.6, -3.9, 0.1)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(5.0),

            // Drive to front of target
            new ProfiledDistanceDriveCommand(180, 0.5, -1.05, -0.70)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(2.0),

            // Score
            Commands.run(Subsystems.intake::eject).withTimeout(0.25),

            //
            // Start third pickup
            // 
            
            new ProfiledDistanceDriveCommand(180, 0.4, 1, 0.55)
                .withEndSpeed(0.5)
                .withTimeout(3),
            
            new ProfiledDistanceDriveCommand(180, 1, 2.5, 0)
                .withEndSpeed(0.2)
                .withTimeout(3),

            new RotateToAngle(-45).withTimeout(2.0),

            new SchedulePose(Pose.GroundPickup),
            
            Commands.parallel(
                new ProfiledDistanceDriveCommand(-45, 0.2, 1.8, -2)
                    .withEndSpeed(0.5)
                    .withThreshold(0.1)
                    .withTimeout(pickupTimeout),
                new ClampHandOnPart().withTimeout(pickupTimeout)
            ).withTimeout(pickupTimeout)
      
        );
    }
    
}
