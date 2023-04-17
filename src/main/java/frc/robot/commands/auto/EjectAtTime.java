package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class EjectAtTime extends CommandBase {

    private double timeThreshold = 0.75;

    public EjectAtTime() {
    }

    public EjectAtTime withTimeThreshold(double threshold) {
        this.timeThreshold = threshold;
        return this;
    } 
    
    @Override
    public void execute() {
        if (Timer.getMatchTime() < timeThreshold) {
            System.out.println("!!! END OF AUTO EJECT !!!");
            Subsystems.intake.eject();
        }
    }
}
