package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.WristPosition;;

public class PoseWrist extends CommandBase
 {
  
  private final WristPosition position;

public PoseWrist(Intake.WristPosition position){
    this.position = position;
  }
    @Override
   public void execute() {
       Subsystems.intake.setWristPosition(position);
   } 
}
