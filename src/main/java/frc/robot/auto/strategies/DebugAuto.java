// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.strategies;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DebugAuto extends SequentialCommandGroup {
  public DebugAuto() {
    // double speed = 0.5;
    // double offset = -90.0;
    // addCommands(
    //   new InitializeAutoState(45, ShooterProfile.Dynamic),
    //   new ProfiledDistanceDriveCommand(45, 0.25, -1, 0).withRobotCentric().withTimeout(3.0),
    //   new ProfiledDistanceDriveCommand(45, 0.25, 0, -1).withRobotCentric().withTimeout(3.0)
    // );
  }

  // private CommandGroupBase driveSquare(double angle, double speed) {
  //   return CommandGroupBase.sequence(
  //     new ProfiledDistanceDriveCommand(angle, speed, 1, 0),
  //     new ProfiledDistanceDriveCommand(angle, speed, 0, 1),
  //     new ProfiledDistanceDriveCommand(angle, speed, -1, 0),
  //     new ProfiledDistanceDriveCommand(angle, speed, 0, -1)

  //     // new SimpleDistanceDriveCommand(0, speed, 1, 0),
  //     // new SimpleDistanceDriveCommand(0, speed, 0, 1),
  //     // new SimpleDistanceDriveCommand(0, speed, -1, 0),
  //     // new SimpleDistanceDriveCommand(0, speed, 0, -1)
  //   );
  // }
}
