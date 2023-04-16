package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.EnableImageProcessing;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Pipeline;

public class VisionTestStrategy extends SequentialCommandGroup {

    private VisionAlignmentHelper visionHelper = new VisionAlignmentHelper();

    public VisionTestStrategy() {
        this(Pipeline.Cone);
    }
    public VisionTestStrategy(Pipeline pipeline) {
        addCommands(
            new InitializeAutoState(0),
            // new InstantCommand(Subsystems.intake::CloseHand),
            new EnableImageProcessing(pipeline)
        );

        addCommands(
            new ProfiledDistanceDriveCommand(0, 0.3, 2, 0)
                .withXSupplier(visionHelper::calculate)
                .withTimeout(5)
        );
    }
    
}
