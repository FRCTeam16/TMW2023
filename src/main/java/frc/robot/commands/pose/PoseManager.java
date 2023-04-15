package frc.robot.commands.pose;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PoseManager {
    public enum Pose {
        StartingConfig,
        SingleSubstation,
        DoubleSubstation,
        PreGroundPickup,
        GroundPickup,
        Stow,
        ScoreMidCone,
        ScoreHighCone,
        Zero,
        Shoot
    }

    private Pose currentPose = Pose.StartingConfig;
    private Pose lastPose = Pose.StartingConfig;
    private Map<Pose, Supplier<Command>> commandRegistry = new HashMap<>();

    public PoseManager() {
        commandRegistry.put(Pose.PreGroundPickup, MoveToPreGroundPickupPose::new);
        commandRegistry.put(Pose.GroundPickup, MoveToGroundPickupPose::new);
        commandRegistry.put(Pose.Stow, MoveToStowPose::new);
        commandRegistry.put(Pose.SingleSubstation, MoveToSingleSubstationPose::new);
        commandRegistry.put(Pose.DoubleSubstation, MoveToDoubleSubstationPose::new);
        commandRegistry.put(Pose.ScoreMidCone, MoveToScoreConeMidPose::new);
        commandRegistry.put(Pose.ScoreHighCone, MoveToScoreConeHighPose::new);
        commandRegistry.put(Pose.Zero, MoveToZeroPose::new);
        commandRegistry.put(Pose.Shoot, MoveToShootPose::new);
    }

    public Command getPose(Pose requestedPose) {
        // TODO: make dynamic registry for illegal moves
        System.out.println("PoseManager getPose: " + currentPose + " => " + requestedPose);
        boolean illegal = false;

        // Can add state checks of currentPose vs. requestedPose to prevent movements

        if (DriverStation.isTeleop()) {
            if (illegal) {
                return Commands.print("!!! [PoseManager] ILLEGAL MOVE REQUESTED: " + currentPose + " -> " + requestedPose);
            }
        } else if (DriverStation.isAutonomous()) {
            System.out.println("Autonomous Override of Illegal Move: " + currentPose + " -> " + requestedPose);
        }

        // Look up pose
        if (commandRegistry.containsKey(requestedPose)) {
            lastPose = currentPose;
            currentPose = requestedPose;
            return commandRegistry.get(requestedPose).get();
        } else {
            return Commands.print("!!! [PoseManager] Unhandled pose requested: " + requestedPose);
        }
    }

    public Pose getCurrentPose() {
        return this.currentPose;
    }

    public Pose getLastPose() {
        return this.lastPose;
    }
}
