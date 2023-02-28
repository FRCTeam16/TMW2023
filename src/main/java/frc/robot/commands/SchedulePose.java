package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems;
import frc.robot.commands.pose.PoseManager.Pose;

public class SchedulePose extends CommandBase {

    private Pose pose;

    public SchedulePose(Pose pose) {
        this.pose = pose;
    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(pose));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
