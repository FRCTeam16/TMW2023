package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;

public class PDistTest extends SequentialCommandGroup {

    public PDistTest() {
        addCommands(
            new ProfiledDistanceDriveCommand(0, 0.2, 0.5, 0)
        );
    }
    
}
