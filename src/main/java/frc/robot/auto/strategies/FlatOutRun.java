package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

public class FlatOutRun extends SequentialCommandGroup {

    public FlatOutRun() {
       
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand),
            new WaitCommand(0.5),

            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1.0),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.5),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint),

            new SchedulePose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            new SchedulePose(Pose.Stow),


            new WaitCommand(5),
            

            // Do drive
            new ProfiledDistanceDriveCommand(180, 0.5, 0.1, -1.25)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(3.0),
            new ProfiledDistanceDriveCommand(180, 0.5, 2.5, 0)
            
        );;
    }
}
