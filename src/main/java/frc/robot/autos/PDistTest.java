package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;

public class PDistTest extends SequentialCommandGroup {

    public PDistTest() {
        addCommands(
            new InitializeAutoState(180),
            new ProfiledDistanceDriveCommand(180, 0.3, -1, 0)
        );
    }
    
}
