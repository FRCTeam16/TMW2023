package frc.robot;

import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class changePipeline extends CommandBase{

    Limelight limelight = new Limelight();

    public changePipeline(Pipelines pipe) {
        changePip(pipe);
    }

    public void togglePip() {
        if(limelight.getCurrentPipelin() == 0){
            changePip(Pipelines.April);
            return;
        }
        if(limelight.getCurrentPipelin() == 1){
            changePip(Pipelines.Retro);
            return;
        }
        else{
            pipThrow();
        }
    }
    
    public void changePip(Pipelines pipe){
        if(pipe == Pipelines.Retro){
            //set to pipeline one
            limelight.setCurrentPipeline(0);
            return;
        }
        if(pipe == Pipelines.April){
            //set to pipeline two
            limelight.setCurrentPipeline(1);
            return;
        }
        else {
            pipThrow();
        }
    }

    public enum Pipelines{
        Retro,
        April
    }

    private void pipThrow(){
        System.out.println("Invalid pipline type passed, try passing Pipelines.Retro (for pipeline 0) or Pipelines.April (for pipeline 1)");
    }



}
