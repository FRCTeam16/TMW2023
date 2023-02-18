package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.PivotPosition;

public class PosePivot extends CommandBase
 {
  
  private final PivotPosition position;

public PosePivot(Pivot.PivotPosition position){
    this.position = position;
  }
    @Override
   public void execute() {
       Subsystems.pivot.setPivotPosition(position);
   } 
}
