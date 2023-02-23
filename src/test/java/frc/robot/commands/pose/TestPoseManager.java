package frc.robot.commands.pose;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.pose.PoseManager.Pose;
import org.junit.jupiter.api.Test;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;;

public class TestPoseManager {

    @Test
    public void testIllegalMoveStowHigh() {
        PoseManager poseManager = new PoseManager();
        poseManager.getPose(Pose.Stow);
        Command result  = poseManager.getPose(Pose.ScoreHighCone);
        assert(result instanceof PrintCommand);
    }

    // @Test
    // public void testIllegalMoveHighStow() {
    //     PoseManager poseManager = new PoseManager();
    //     poseManager.getPose(Pose.ScoreHighCone);
    //     Command result  = poseManager.getPose(Pose.Stow);
    //     assert(result instanceof PrintCommand);
    // }
    
}
