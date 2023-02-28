package frc.robot.auto.strategies;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.TrajectoryDriveFactory;

public class TestTrajectoryFactory extends SequentialCommandGroup {
    public TestTrajectoryFactory() {
        addCommands(
            new InitializeAutoState(180),
            // Subsystems.poseManager.getPose(Pose.Stow), 
            // new WaitCommand(1.0), 
            TrajectoryDriveFactory.createCommand(backwardsTest())
            // TrajectoryDriveFactory.createCommand(driveOntoChargeStation())
        );
    }

    private double ONEEIGHTY_RADS = Math.toRadians(180);

    Trajectory backwardsTest() {
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(ONEEIGHTY_RADS)),
                List.of(
                   new Translation2d(2, 0)
                    ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3.0, 0, new Rotation2d(ONEEIGHTY_RADS)),
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setReversed(true)
                    .setKinematics(Constants.Swerve.swerveKinematics));
    }

    
    Trajectory driveToBackOfChargeStation() {
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(ONEEIGHTY_RADS)),
                List.of(
                   new Translation2d(4, 0)
                    ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.0, 2.2, new Rotation2d(ONEEIGHTY_RADS)),
                new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setReversed(true)
                    .setKinematics(Constants.Swerve.swerveKinematics));
    }

    Trajectory driveOntoChargeStation() {
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(4, 2.2, new Rotation2d(0)),
                List.of(),
                new Pose2d(1.75, 2.2, new Rotation2d(0)),
                new TrajectoryConfig(
                    0.5, 0.5)
                    .setReversed(true)
                    .setKinematics(Constants.Swerve.swerveKinematics));
    }
    
}
