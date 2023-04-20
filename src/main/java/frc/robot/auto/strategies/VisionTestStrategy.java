package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.EnableImageProcessing;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.auto.ClampHandOnPart;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.PartIndicator.PartType;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Pipeline;

public class VisionTestStrategy extends SequentialCommandGroup {

    private VisionAlignmentHelper visionHelper = new VisionAlignmentHelper();

    public VisionTestStrategy() {
        this(Pipeline.Cone);
    }

    public VisionTestStrategy(Pipeline pipeline) {
        boolean isCone = pipeline == Pipeline.Cone;
        addCommands(
            new InitializeAutoState(0),
            new InstantCommand(() -> Subsystems.swerveSubsystem.hardResetOdometry(0)),
            new WaitCommand(.1),
            // new InstantCommand(Subsystems.intake::CloseHand),
            new EnableImageProcessing(pipeline)
        );

        addCommands(
             //important
             new SchedulePose(Pose.AutoPreGroundPickup),
             new WaitCommand(0.5),
             new VisionAlign()
                 .withVisionPipeline(pipeline)
                 .withRobotAngle(0)
                 .withTolerance(6)
                 .withRobotSpeed(0.5)
                 .withTimeout(1.5),
 
             new InstantCommand(() -> Subsystems.partIndicator.requestPart(isCone ? PartType.Cone : PartType.Cube)),
             new SchedulePose(Pose.GroundPickup),
             new InstantCommand(Subsystems.intake::intake),
             new InstantCommand(Subsystems.intake::OpenHand),
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
             new PrintCommand("Finished pickup")
        );

        
    }
    
}
