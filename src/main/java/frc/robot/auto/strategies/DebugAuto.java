// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.InitializeAutoState;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    // double speed = 0.5;
    // double offset = -90.0;
    addCommands(
      new InitializeAutoState(180)
    );
  }
}
