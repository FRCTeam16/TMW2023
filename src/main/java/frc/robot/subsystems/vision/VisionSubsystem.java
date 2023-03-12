package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.Limelight.CameraMode;
import frc.robot.subsystems.vision.Limelight.LEDMode;
import frc.robot.subsystems.vision.Limelight.SceneInfo;

/**
 * Vision subsystem.  
 */
public class VisionSubsystem extends SubsystemBase implements Lifecycle {
  private final Limelight limelight;
  private VisionInfo visionInfo = new VisionInfo();

  private final double CAMERA_HEIGHT_IN;
  private final double TARGET_HEIGHT_IN;
  private final double CAMERA_ANGLE_DEGREES = 28.0;


  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    limelight = new Limelight();
    double defaultCameraHeight = 36.5;
    double defaultTargetHeight = 104.0;
    
    SmartDashboard.setDefaultNumber("Vision/CameraHeightInches", defaultCameraHeight);
    SmartDashboard.setDefaultNumber("Vision/TargetHeightInches", defaultTargetHeight);
    CAMERA_HEIGHT_IN = SmartDashboard.getNumber("Vision/CameraHeightInches", defaultCameraHeight);
    TARGET_HEIGHT_IN = SmartDashboard.getNumber("Vision/TargetHeightInches", defaultTargetHeight);
  }

  public Limelight getLimelight() {
    return limelight;
  }

  public VisionInfo getVisionInfo() {
    return visionInfo;
  }

  public void enable() {
    limelight.setCameraMode(CameraMode.ImageProcessing);
    limelight.setLEDMode(LEDMode.CurrentPipeline);
  }

  public void disable() {
    limelight.setLEDMode(LEDMode.ForceOff);
  }

  @Override
  public void periodic() {
    var sceneInfo = limelight.getScene();
    var aprilInfo = limelight.getAprilInfo();
    
    // Update out information
    visionInfo = new VisionInfo();
    visionInfo.hasTarget = sceneInfo.hasTarget;
    visionInfo.xOffset = sceneInfo.xOffset;
    visionInfo.yOffset = sceneInfo.yOffset;
    visionInfo.targetArea = sceneInfo.targetArea;

    visionInfo.targetId = aprilInfo.targetId;
 
    double distance_inches = -1.0;
    if (visionInfo.hasTarget) {
      distance_inches = CalculateDistance(CAMERA_HEIGHT_IN, TARGET_HEIGHT_IN, CAMERA_ANGLE_DEGREES, visionInfo.yOffset);
    }
    visionInfo.distanceToTarget = distance_inches;
    
    SmartDashboard.putNumber("Vision/DistToTargetInches", visionInfo.distanceToTarget);

    if(CheckValidPosition(CAMERA_HEIGHT_IN, TARGET_HEIGHT_IN, CAMERA_ANGLE_DEGREES, visionInfo.yOffset)){
      
    }
  }

  /**
   * Calculates the distance to the currently observed target, or -1 if no target
   * 
   * @param heightToCamera 
   * @param heightToTarget
   * @return
   */
  public double CalculateDistance(double heightToCamera, double heightToTarget, double cameraAngle, double yOffset) {
    if (this.visionInfo.hasTarget) {
      double goalRadians = Units.degreesToRadians(cameraAngle + yOffset);
      return (heightToTarget - heightToCamera) / Math.tan(goalRadians);
    } else {
      return -1;
    }
}

  /**
   * Vision Information
   */
  public static class VisionInfo extends SceneInfo {
    public double distanceToTarget = -1;
  }

  public boolean CheckCentered(){
    double X = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    if(X > -0.1 && X < 0.71 ){
      return true;
    }
    return false;
  }

  public boolean CheckValidPosition(double heightToCamera, double heightToTarget, double cameraAngle, double yOffset) {
    if(CheckCentered()){
      if(CalculateDistance(heightToCamera, heightToTarget, cameraAngle, yOffset) < 4){
        return true;
      }
    }
    return false;
  }

  public enum Pipeline {
    April(0),
    Retro(1);

    public final int pipelineNumber;

    private Pipeline(int number) {
        this.pipelineNumber = number;
    }
  }

  public CommandBase selectPipeline(Pipeline pipeline) {
    return Commands.runOnce(() -> limelight.setCurrentPipeline(pipeline.pipelineNumber))
      .ignoringDisable(true);
  }
}
