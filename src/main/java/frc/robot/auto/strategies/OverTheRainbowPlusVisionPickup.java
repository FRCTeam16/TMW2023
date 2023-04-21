package frc.robot.auto.strategies;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.Balance;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.XWHeelLock;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.auto.StopDrive;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.PartIndicator.PartType;
import frc.robot.subsystems.vision.Pipeline;

public class OverTheRainbowPlusVisionPickup extends SequentialCommandGroup {

    private PitchDropWatcher pitchWatcher = new PitchDropWatcher();
    private StopAfterDriveOverWatcher driveWatcher = new StopAfterDriveOverWatcher();

    public OverTheRainbowPlusVisionPickup() {
        this(Pipeline.Cone);
    }

    public OverTheRainbowPlusVisionPickup(Pipeline visionPipeline) {
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );

        ScoreHighHelper.scoreHighCone(this);

        boolean isCone = visionPipeline == Pipeline.Cone;

        addCommands(
            // new SchedulePose(Pose.SingleSubstation),
            // new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),
            new WaitCommand(0.5),

            // Offset
            new ProfiledDistanceDriveCommand(180, 0.3, 0.25, 0)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(2.0),

            // Over
            new ProfiledDistanceDriveCommand(180, 0.9, 3.75, 0)
                .withEndSpeed(0.3)
                .withStopCondition(this.driveWatcher::isFinished)
                .withThreshold(0.1)
                .withTimeout(8.0),

            new StopDrive(),
            new InstantCommand(() -> Subsystems.intake.intake()),

            // Spin to face target
            new RotateToAngle(0)
                .withThreshold(10)
                .withTimeout(1.5),

            //important
            new SchedulePose(Pose.AutoPreGroundPickup),
            new VisionAlign()
                .withVisionPipeline(visionPipeline)
                .withRobotAngle(0)
                .withTolerance(6)
                .withRobotSpeed(0.5)
                .withTimeout(1.0),

            new InstantCommand(() -> Subsystems.partIndicator.requestPart(isCone ? PartType.Cone : PartType.Cube)),
            new SchedulePose(Pose.GroundPickup),
            new WaitCommand(0.5),


            // Pickup
            new PrintCommand("Starting pickup"),
            Commands.race(
                new ProfiledDistanceDriveCommand(0, 0.25, .7, 0)
                    .withEndSpeed(0.25)
                    .withThreshold(0.1)
                    .withRobotCentric()
                    .withTimeout(2),
                new ClampHandOnPart(isCone)
            ).withTimeout(2),
            isCone ? new InstantCommand(() -> Subsystems.intake.CloseHand()) : new PrintCommand("Skip close for cube"),
            new PrintCommand("Finished pickup"),

            // Spin
            // Commands.parallel(
            //     new RotateToAngle(180)
            //         .withThreshold(5)
            //         .withTimeout(1.5),
            //     new SchedulePose(Pose.Stow)
            // ),
            new SchedulePose(Pose.Stow),

            // Drive onto ramp
            new PrintCommand("Running onto ramp for balance"),
            new ProfiledDistanceDriveCommand(0, 0.5, -2.0, 0)         // 3.1 for drivethru non race pickup
                // .withStopCondition(this::stopOPitch)
                .withEndSpeed(0.5)
                .withTimeout(3.0),

            // Drive until we see our pitch
            // new ProfiledDistanceDriveCommand(180, 0.25, -3.5, 0)
            //     .withEndSpeed(0.25)
            //     .withStopCondition(this.pitchWatcher::shouldStopNoMaxWatch)
            //     .withTimeout(8.0),
            new InstantCommand(() -> Subsystems.partIndicator.requestPart(!isCone ? PartType.Cone : PartType.Cube)),
            new PrintCommand("*** Starting balance"),
            new Balance()
            // new XWHeelLock()
        );
    }

    public static class PitchDropWatcher {
        boolean watchDropMode = false;
        double angle = 13.0;

        PitchDropWatcher() {
        }

        PitchDropWatcher(double angle) {
            this.angle = angle;
        }

        public boolean shouldStopNoMaxWatch() {
            double pitch = Subsystems.swerveSubsystem.gyro.getPitch();

             if (pitch < angle) {
                return true;
            } else {
                return false;
            }
        }
    }
    

    private static class StopAfterDriveOverWatcher {
        private boolean sawDriveUp;
        private boolean sawDriveDown;
        private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
        private Timer endTimer = new Timer();

        StopAfterDriveOverWatcher() {}

        public boolean isFinished() {
            double pitch = filter.calculate(Subsystems.swerveSubsystem.gyro.getPitch());
            if (!sawDriveUp) {
                if (pitch < -10) {
                    sawDriveUp = true;
                }
            } else if (!sawDriveDown) {
                if (pitch > 10) {
                    sawDriveDown = true;
                    endTimer.start();
                }
            } else {
                // we have driven up and down
                // TODO: determine if we need a small drive forward timer at the end
                if (Math.abs(pitch) < 2.75 /*&& endTimer.hasElapsed(0.5)*/) {
                    return true;
                }
            }
            return false;
        }
    }

}

