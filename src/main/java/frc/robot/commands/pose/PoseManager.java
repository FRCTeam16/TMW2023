package frc.robot.commands.pose;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// no stow to high
// no high to stow

public class PoseManager {
    public enum Pose {
        StartingConfig,
        SingleSubstation,
        DoubleSubstation,
        GroundPickup,
        Stow,
        ScoreMidCone,
        ScoreHighCone,
        Zero
    }

    private Pose currentPose = Pose.StartingConfig;
    private Map<Pose, Supplier<Command>> commandRegistry = new HashMap<>();

    public PoseManager() {
        commandRegistry.put(Pose.GroundPickup, MoveToGroundPickupPose::new);
        commandRegistry.put(Pose.Stow, MoveToStowPose::new);
        commandRegistry.put(Pose.SingleSubstation, MoveToSingleSubstationPose::new);
        commandRegistry.put(Pose.DoubleSubstation, MoveToDoubleSubstationPose::new);
        commandRegistry.put(Pose.ScoreMidCone, MoveToScoreConeMidPose::new);
        commandRegistry.put(Pose.ScoreHighCone, MoveToScoreConeHighPose::new);
        commandRegistry.put(Pose.Zero, MoveToZeroPose::new);
    }

    public Command getPose(Pose requestedPose) {
        // TODO: make dynamic registry for illegal moves
        System.out.println("PoseManager getPose: " + currentPose + " => " + requestedPose);
        boolean illegal = false;
        if (currentPose == Pose.Stow && (requestedPose == Pose.ScoreHighCone || requestedPose == Pose.GroundPickup)) {
            illegal = true;
        } else if (currentPose == Pose.ScoreHighCone && (requestedPose == Pose.Stow || requestedPose == Pose.Zero)) {
            illegal = true;
        }
        else if (currentPose == Pose.GroundPickup && (requestedPose == Pose.Stow || requestedPose == Pose.Zero || requestedPose == Pose.SingleSubstation || requestedPose == Pose.DoubleSubstation)) {
            illegal = true;
        }
        if (illegal) {
            return Commands.print("!!! [PoseManager] ILLEGAL MOVE REQUESTED: " + currentPose + " -> " + requestedPose);
        }
        if (commandRegistry.containsKey(requestedPose)) {
            currentPose = requestedPose;
            return commandRegistry.get(requestedPose).get();
        } else {
            return Commands.print("!!! [PoseManager] Unhandled pose requested: " + requestedPose);
        }
    }

    public Pose getCurrentPose() {
        return this.currentPose;
    }
}
