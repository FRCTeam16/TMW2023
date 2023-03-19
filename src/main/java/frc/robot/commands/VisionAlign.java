package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Pipeline;

public class VisionAlign extends CommandBase {
    private VisionAlignmentHelper helper = new VisionAlignmentHelper();
    private double robotAngle = 180.0;

    public VisionAlign() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    public VisionAlign withRobotAngle(double angle) {
        this.robotAngle = angle;
        return this;
    }

    public VisionAlign withTolerance(double tolerance) {
        this.helper.setTolerance(tolerance);
        return this;
    }
    
    @Override
    public void initialize() {
        Subsystems.visionSubsystem.getLimelight().setCameraMode(CameraMode.ImageProcessing);
        // FIXME: NOT WORKING Subsystems.visionSubsystem.selectPipeline(Pipeline.RetroHigh);
        Subsystems.visionSubsystem.getLimelight().setCurrentPipeline(Pipeline.RetroHigh.pipelineNumber);
        Subsystems.visionSubsystem.getLimelight().setLEDMode(LEDMode.CurrentPipeline);
    }


    @Override
    public void execute() {
        RotationController controller = Subsystems.swerveSubsystem.getRotationController();
        double twist = controller.calculate(
                Subsystems.swerveSubsystem.getYaw().getDegrees(), robotAngle);

        Translation2d translation = new Translation2d(0,  -this.helper.calculate());

        Subsystems.swerveSubsystem.drive(
            translation.times(Constants.Swerve.maxSpeed / 4), 
            Math.toRadians(twist), 
            true, 
            true);
    }

    @Override
    public boolean isFinished() {
        // return this.pid.atSetpoint();
        return false;
    }
}
