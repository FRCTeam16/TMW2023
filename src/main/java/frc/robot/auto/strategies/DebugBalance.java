package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.Balance;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

/**
 * Measure 53" from hinge to frame of robot
 */
public class DebugBalance extends SequentialCommandGroup {
    public DebugBalance() {

        //
        // Copied from OTR+Vision
        // 
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand)
        );

        addCommands(

            // Spin
            new SchedulePose(Pose.Stow),
            new WaitCommand(0.5),

            // Drive onto ramp
            new ProfiledDistanceDriveCommand(180, 0.4, -2.0, 0)         // 3.1 for drivethru non race pickup
                // .withStopCondition(this::stopOPitch)
                .withEndSpeed(0.4)
                .withTimeout(3.0),

            // Drive until we see our pitch
            // new ProfiledDistanceDriveCommand(180, 0.25, -3.5, 0)
            //     .withEndSpeed(0.25)
            //     .withStopCondition(this.pitchWatcher::shouldStopNoMaxWatch)
            //     .withTimeout(8.0),
            // new InstantCommand(() -> Subsystems.partIndicator.requestPart(!isCone ? PartType.Cone : PartType.Cube)),

            new Balance()
        );
    }
}
