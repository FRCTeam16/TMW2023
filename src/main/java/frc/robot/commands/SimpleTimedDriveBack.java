package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;

/**
 * ONLY USED FOR AUTOSCORE
 */
public class SimpleTimedDriveBack extends CommandBase {
    private Timer timer = new Timer();
    private double timeToDrive = 0.0;

    public SimpleTimedDriveBack(double timeToDrive) {
        this.timeToDrive = timeToDrive;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        double twist = controller.calculate(
                Subsystems.swerveSubsystem.getYaw().getDegrees(), 180);
        Translation2d translation = new Translation2d(-0.5, -0);
        Subsystems.swerveSubsystem.drive(
            translation.times(Constants.Swerve.maxSpeed / 4), 
            Math.toRadians(twist), 
            true, 
            true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeToDrive);
    }
}