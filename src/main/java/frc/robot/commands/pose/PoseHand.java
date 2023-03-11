package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake.HandState;

public class PoseHand extends CommandBase {
    private final HandState setState;

    public PoseHand(HandState setState) {
        this.setState = setState;
    }

    @Override
    public void execute() {
        if(setState == HandState.Open) {
            Subsystems.intake.OpenHand();
        }
        else {
            Subsystems.intake.CloseHand();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
