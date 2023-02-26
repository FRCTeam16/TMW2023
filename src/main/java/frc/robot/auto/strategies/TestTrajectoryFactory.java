package frc.robot.auto.strategies;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.TrajectoryDriveFactory;

public class TestTrajectoryFactory extends SequentialCommandGroup {
    public TestTrajectoryFactory() {
        addCommands(
            TrajectoryDriveFactory.createCommand(createTrajectory())
        );
    }

    Trajectory createTrajectory() {
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(0.5, 0), 
                    new Translation2d(0.5, -1)
                    ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.5, -2, new Rotation2d(0)),
                TrajectoryDriveFactory.DEFAULT_TRAJECTORY_CONFIG);
    }
    
}
