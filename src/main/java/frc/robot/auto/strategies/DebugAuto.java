// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    // double speed = 0.5;
    // double offset = -90.0;
    addCommands(
      new InitializeAutoState(0));
    //   new SchedulePose(Pose.Stow),
    //   new ProfiledDistanceDriveCommand(0, 0.5, 3, 0)
    //     .withEndSpeed(0),
    //     new ProfiledDistanceDriveCommand(0, 0, 0, 0)
    // );
  }
}
