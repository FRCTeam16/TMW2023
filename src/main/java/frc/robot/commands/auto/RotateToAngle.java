package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
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

    public RotateToAngle withThreshold(double threshold) {
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        controller.setTolerance(threshold);
        return this;
    }

    @Override
    public void execute() {
        double clamp = SmartDashboard.getNumber("AutoRotationClamp", 6);
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        double twist = controller.calculate(
            //Subsystems.swerveSubsystem.getPose().getRotation().getDegrees(), 
            Subsystems.swerveSubsystem.getYaw().getDegrees(), 
            this.targetAngleDegrees);
        
        Subsystems.swerveSubsystem.drive(
            new Translation2d(0,0), 
            MathUtil.clamp(Math.toRadians(twist), -clamp, clamp),
            true,
            true);

        SmartDashboard.putNumber("RotateToAngleError", Subsystems.swerveSubsystem.getRotationController().getPositionError());
    }

    @Override
    public boolean isFinished() {
        return Subsystems.swerveSubsystem.getRotationController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.swerveSubsystem.getRotationController().resetTolerance();
    }
    
}
