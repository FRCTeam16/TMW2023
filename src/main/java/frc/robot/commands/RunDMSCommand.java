package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class RunDMSCommand extends CommandBase {
  /** Creates a new RunDMSCommand. */
  public RunDMSCommand() {
    addRequirements(Subsystems.ledSubsystem, Subsystems.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // FIXME Subsystems.drivetrainSubsystem.setDmsMode(true);
    Subsystems.ledSubsystem.startDMS();
  }

  @Override
  public void end(boolean interrupted) {
    // FIXME Subsystems.drivetrainSubsystem.setDmsMode(false);
    Subsystems.ledSubsystem.stopDMS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("DMS isFinished");
    return Subsystems.ledSubsystem.isStopped() || DriverStation.isDisabled();
  }
}
