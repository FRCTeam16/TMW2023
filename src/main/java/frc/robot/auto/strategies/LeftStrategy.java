package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager;
import frc.robot.commands.pose.PoseManager.Pose; 

public class LeftStrategy extends SequentialCommandGroup {

    public final double LSpeed = 0.1;
    public final int LEspeed = 2;

    public LeftStrategy(){
        addCommands(
            new InitializeAutoState(180),
            new ProfiledDistanceDriveCommand(0, LSpeed, 0, -0.5)
                .withEndSpeed(LSpeed/2)
                .withTimeout(1),
            new SchedulePose(Pose.ScoreHighCone),
            new WaitCommand(1),
            new InstantCommand(() -> Subsystems.intake.OpenHand()),
            new InstantCommand(() -> Subsystems.intake.eject()),

            new SchedulePose(Pose.Zero),

            new ProfiledDistanceDriveCommand(0, LSpeed, 0, 3)
                .withEndSpeed(LSpeed/LEspeed)
                .withTimeout(1)
            
        );
    }
}
