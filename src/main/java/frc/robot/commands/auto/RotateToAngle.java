package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;

public class RotateToAngle extends CommandBase {
    private final double targetAngleDegrees;

    public RotateToAngle(double targetAngleDegrees) {
        this.targetAngleDegrees = targetAngleDegrees;
    }

    @Override
    public void execute() {
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        double twist = controller.calculate(
            //Subsystems.swerveSubsystem.getPose().getRotation().getDegrees(), 
            Subsystems.swerveSubsystem.getYaw().getDegrees(), 
            this.targetAngleDegrees);
        
        Subsystems.swerveSubsystem.drive(
            new Translation2d(0,0), 
            Math.toRadians(twist), 
            true,
            true);

        SmartDashboard.putNumber("RotateToAngleError", Subsystems.swerveSubsystem.getRotationController().getPositionError());
    }

    @Override
    public boolean isFinished() {
        return Subsystems.swerveSubsystem.getRotationController().atSetpoint();
    }
    
}