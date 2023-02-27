package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.ZeroAndSetOffsetCommand;

public class InitializeAutoState extends ParallelCommandGroup {

  /** Creates a new InitializeAutoState. */
  public InitializeAutoState(double initialRobotAngleDegrees) {
    addCommands(
      new InstantCommand(() -> Subsystems.swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),
      new ZeroAndSetOffsetCommand(initialRobotAngleDegrees)
    );
  }
}
