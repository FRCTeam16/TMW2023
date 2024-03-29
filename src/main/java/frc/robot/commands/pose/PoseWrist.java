package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake.WristPosition;

public class PoseWrist extends CommandBase {
    private final WristPosition position;

    public PoseWrist(WristPosition position) {
        this.position = position;
    }

    @Override
    public void execute() {
        Subsystems.intake.setWristPosition(position);
        
        if(position == WristPosition.DoubleSubstation || position == WristPosition.SingleSubstation) {
            Subsystems.intake.setAtSubstation(true);
        }
        else {
            Subsystems.intake.setAtSubstation(false);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
