package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;

public class TestRotationController extends CommandBase {
    private double targetAngleDegrees;

    public TestRotationController(double targetAngleDegrees) {
        this.targetAngleDegrees = 
            (Subsystems.swerveSubsystem.getPose().getRotation().getDegrees() + targetAngleDegrees) % 360;
    }

    @Override
    public void execute() {
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        double twist = controller.calculate(
            Subsystems.swerveSubsystem.getPose().getRotation().getDegrees(), 
            this.targetAngleDegrees);
        
        Subsystems.swerveSubsystem.drive(
            new Translation2d(0,0), 
            Math.toRadians(twist), 
            true,
            true);
    }
    
    @Override
    public boolean isFinished() {
        return Subsystems.swerveSubsystem.getRotationController().atSetpoint();
    }
    
}
