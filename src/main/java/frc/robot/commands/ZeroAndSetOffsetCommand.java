package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class ZeroAndSetOffsetCommand extends CommandBase {
  private double offsetAngleDegrees;

  /** Creates a new ZeroAndSetOffsetCommand. */
  public ZeroAndSetOffsetCommand(double offsetAngleDegrees) {
    this.offsetAngleDegrees = offsetAngleDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ZeroANdSetOffsetCommand(" + offsetAngleDegrees + ") called");
    Subsystems.swerveSubsystem.gyro.zeroGyroscope();
    Subsystems.swerveSubsystem.gyro.setGyroOffset(offsetAngleDegrees);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
