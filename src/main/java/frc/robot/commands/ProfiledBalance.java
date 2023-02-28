package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.Subsystems;

/**
 * Untested
 */
public class ProfiledBalance extends ProfiledPIDCommand {

    public ProfiledBalance() {
        super(
            new ProfiledPIDController(3, 0, 0, 
                new TrapezoidProfile.Constraints(Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity)),
            Subsystems.swerveSubsystem.gyro::getPitch, 
            0.0, 
            (output, setpoint) -> balanceDrive(output, setpoint), 
            Subsystems.swerveSubsystem);
    }
    
    private static void balanceDrive(double output, TrapezoidProfile.State setpoint) {
        Translation2d translation = new Translation2d(output, 0).times(Constants.Swerve.maxSpeed / 8);
        Subsystems.swerveSubsystem.drive(translation, 0, true, true);
    }
}
