package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class boto extends CommandBase {
    double Rx;
    double Ry;
    int i;

    public boto(double Rx, double Ry){ // boto test constructor
        System.out.println(Rx + " - " + Ry);
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void initialize(){
        i = 0;
    }

    @Override
    public void execute(){
        System.out.println(Rx + " " +  Ry);
        i++;
    }

    @Override 
    public boolean isFinished(){
        if(i>100){
            return true;
        }
        return false;
    }

    
    
}
