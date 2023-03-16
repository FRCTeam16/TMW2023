package frc.robot.subsystems;

import frc.robot.Subsystems;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PositionToPose {
    
    Rotation2d currentRotation = Subsystems.swerveSubsystem.gyro.getGyroscopeRotation();
    double currentRotationDeg = currentRotation.getDegrees();

  public boolean CheckCentered(){
    double X = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    if(X > -0.1 && X < 0.71 ){
      return true;
    }
    return false;
  }

  public boolean checkRotation(){
    if(currentRotationDeg > 178 && currentRotationDeg < 182){
        return true;
    }
    return false;
  }

  // we need somthing to check distance IDK if that shoud be doable by limelight or what
  
}