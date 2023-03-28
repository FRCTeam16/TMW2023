package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class ClampHandOnPart extends CommandBase {
    private double sawPartTime = 0;
    private boolean sawPart = false;

    @Override
    public void execute() {
        // if (Subsystems.intake.hasPart()) {
        //     Subsystems.intake.CloseHand();
        //     sawPart = true;
        //     sawPartTime = Timer.getFPGATimestamp();
        // }
        if (Subsystems.intake.hasPart()) {
            Subsystems.intake.CloseHand();
            sawPart = true;
            // sawPartTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        // boolean result =  sawPart && (Timer.getFPGATimestamp() - sawPartTime) > 0.5;
        // if (result) {
        //     System.out.println("ClampHandOnPart success");
        // }
        // return result;
        return sawPart;
    }
    
}
