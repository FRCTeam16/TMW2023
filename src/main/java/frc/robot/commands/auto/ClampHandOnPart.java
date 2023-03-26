package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class ClampHandOnPart extends CommandBase {
    boolean clamped = false;

    @Override
    public void execute() {
        if (Subsystems.intake.hasPart()) {
            Subsystems.intake.CloseHand();
            clamped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return clamped;
    }
    
}
