package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.XWHeelLock;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.auto.ScoreHighHelper;
import frc.robot.commands.pose.PoseManager.Pose;

public class OverTheRainbow extends SequentialCommandGroup {

    private PitchDropWatcher pitchWatcher = new PitchDropWatcher();

    public OverTheRainbow() {
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );

        ScoreHighHelper.scoreHighCone(this);

        addCommands(
            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),

            new ProfiledDistanceDriveCommand(180, 0.3, 0.25, 0.5)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(8.0),

            new ProfiledDistanceDriveCommand(180, 0.6, 4.25, 0)
                .withEndSpeed(0.3)
                .withThreshold(0.1)
                .withTimeout(8.0),

            new ProfiledDistanceDriveCommand(180, 0.4, -1.7, 0)
                // .withStopCondition(this::stopOnPitch)
                .withEndSpeed(0.4)
                .withTimeout(4.0),

            new ProfiledDistanceDriveCommand(180, 0.15, -3.5, 0)
                .withEndSpeed(0.15)
                .withStopCondition(this.pitchWatcher::shouldStopNoMaxWatch)
                .withTimeout(8.0),
            new XWHeelLock()
        );
    }

    public boolean stopOnPitch() {
        return Math.abs(Subsystems.swerveSubsystem.gyro.getPitch()) > 5.0;
    }

    public static class PitchDropWatcher {
        boolean watchDropMode = false;
        double angle = 10;    // THIS IS WHERE WE CHANGE THE ANGLE 
                                // charge station is 60.69 inches from front of grid

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
    
}
