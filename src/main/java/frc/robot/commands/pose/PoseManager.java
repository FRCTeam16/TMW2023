package frc.robot.commands.pose;

import java.sql.Driver;
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


        if (currentPose == Pose.Stow && (requestedPose == Pose.GroundPickup)) {
            illegal = true;
        }
        else if (currentPose == Pose.GroundPickup && (requestedPose == Pose.Stow || requestedPose == Pose.Zero || requestedPose == Pose.SingleSubstation || requestedPose == Pose.DoubleSubstation)) {
            illegal = true;
        }

        if (DriverStation.isTeleop()) {
            if (illegal) {
                return Commands.print("!!! [PoseManager] ILLEGAL MOVE REQUESTED: " + currentPose + " -> " + requestedPose);
            }
        } else if (DriverStation.isAutonomous()) {
            System.out.println("Autonomous Override of Illegal Move: " + currentPose + " -> " + requestedPose);
        }

        // Look up pose
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
