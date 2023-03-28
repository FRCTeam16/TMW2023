package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class StopDrive extends CommandBase {

    public StopDrive() {
    }

    @Override
    public void execute() {
        System.out.println("STOP DRIVE");
        Subsystems.swerveSubsystem.drive(new Translation2d(), 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
