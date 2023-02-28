package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;

public class Balance extends CommandBase implements Lifecycle {
    
    private final double pitchThreshold;
    private final static double BALANCE_TRANSLATION_SPEED = Constants.Swerve.maxSpeed / 8;

    public Balance() {
        this(3.0);
        addRequirements(Subsystems.swerveSubsystem);
    }

    public Balance(double pitchThreshold) {
        this.pitchThreshold = pitchThreshold;
    }

    @Override
    public void execute() {
        final double currentPitch = Subsystems.swerveSubsystem.gyro.getPitch();
        int driveDirection = 0;
        if (currentPitch > pitchThreshold) {
            driveDirection = 1;
        } else if (currentPitch < pitchThreshold) {
            driveDirection = -1;
        }

        Translation2d translation = new Translation2d(0, driveDirection).times(BALANCE_TRANSLATION_SPEED);
        Subsystems.swerveSubsystem.drive(translation, 0, true, true);
    }

}