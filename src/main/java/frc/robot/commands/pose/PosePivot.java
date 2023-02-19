package frc.robot.commands.pose;

import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.subsystems.Pivot.PivotPosition;

public class PosePivot extends CommandBase {

    private final PivotPosition position;

    public PosePivot(PivotPosition position) {
        this.position = position;
    }

    @Override
    public void execute() {
        Subsystems.pivot.setPivotPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
