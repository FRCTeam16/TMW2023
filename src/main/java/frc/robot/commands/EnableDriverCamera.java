package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class EnableDriverCamera extends CommandBase {

    @Override
    public void initialize() {
        Subsystems.visionSubsystem.disable();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
