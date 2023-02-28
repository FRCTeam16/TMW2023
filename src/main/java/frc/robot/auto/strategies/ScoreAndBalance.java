package frc.robot.auto.strategies;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.Balance;
import frc.robot.commands.auto.InitializeAutoState;
import frc.robot.commands.auto.ProfiledDistanceDriveCommand;
import frc.robot.commands.pose.PoseManager.Pose;

public class ScoreAndBalance extends SequentialCommandGroup {
    private double ONEEIGHTY_RADS = Math.toRadians(180);

    public ScoreAndBalance() {
       
        addCommands(
            new InitializeAutoState(180),
            new InstantCommand(Subsystems.intake::CloseHand),
            new WaitCommand(0.5),

            Subsystems.poseManager.getPose(Pose.ScoreHighCone),
            new WaitCommand(1.0),
            new InstantCommand(Subsystems.intake::storeAndScore),
            new WaitCommand(0.25),
            new InstantCommand(Subsystems.intake::OpenHand),
            new WaitCommand(0.5),
            new InstantCommand(Subsystems.intake::restoreStoredSetpoint),

            Subsystems.poseManager.getPose(Pose.SingleSubstation),
            new WaitCommand(0.5),
            Subsystems.poseManager.getPose(Pose.Stow),

            // Do drive
            new ProfiledDistanceDriveCommand(180, 1, 3.75, 0)
                .withEndSpeed(0.5)
                .withThreshold(0.1)
                .withTimeout(5.0),
            new ProfiledDistanceDriveCommand(180, 0.75, 0, 2)
                .withEndSpeed(0.5)
                .withTimeout(2.0),

            new ProfiledDistanceDriveCommand(180, 0.6, -2, 0)
                .withStopCondition(this::stopOnPitch)
                .withTimeout(4.0),
            new Balance(0.5)
            
        );;
    }

    public boolean stopOnPitch() {
        return Math.abs(Subsystems.swerveSubsystem.gyro.getPitch()) > 5.0;
    }

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

    final double RAMP_Y = 2.3;

    Trajectory driveToBackOfChargeStation() {
        // return TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(ONEEIGHTY_RADS)),
        //         List.of(
        //            new Translation2d(4.0, 0)
        //             ),
        //         new Pose2d(4.0, 0, new Rotation2d(ONEEIGHTY_RADS)),
        //         new TrajectoryConfig(
        //                 Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
        //                 Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //             .setReversed(true)
        //             .setKinematics(Constants.Swerve.swerveKinematics));

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

    Trajectory driveOntoChargeStation() {
        return TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(4, RAMP_Y, new Rotation2d(ONEEIGHTY_RADS)),
                List.of(),
                new Pose2d(1.6, RAMP_Y, new Rotation2d(ONEEIGHTY_RADS)),
                new TrajectoryConfig(
                    0.75, 1)
                    .setReversed(false)
                    .setKinematics(Constants.Swerve.swerveKinematics));
    }
    
}
