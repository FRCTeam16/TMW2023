package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Pipeline;

/**
 */
public class EnableImageProcessing extends CommandBase {

    private Pipeline pipeline;

    public EnableImageProcessing(Pipeline pipeline) { 
        this.pipeline = pipeline;
    }

    public void initialize() {
        Subsystems.visionSubsystem.enable();
        Subsystems.visionSubsystem.getLimelight().setCurrentPipeline(pipeline.pipelineNumber);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
