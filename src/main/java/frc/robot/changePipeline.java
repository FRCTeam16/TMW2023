package frc.robot;

import frc.robot.subsystems.vision.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class changePipeline extends CommandBase{

    Limelight limelight = new Limelight();
    
    public void changePip(Pipelines pipe){
        if(pipe == Pipelines.Retro){
            //set to one
            limelight.setCurrentPipeline(0);
        }
        if(pipe == Pipelines.April){
            //set to two
            limelight.setCurrentPipeline(1);
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
        System.out.println("Invalid Pipline type passed, try Retro or April");
    }



}
