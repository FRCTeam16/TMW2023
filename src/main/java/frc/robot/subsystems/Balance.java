package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems;
import frc.robot.subsystems.gyro.BSGyro;

import frc.robot.Constants;

public class Balance extends CommandBase implements Lifecycle {
    //get gyro INFO and do swerve

    

    public Balance(WPI_Pigeon2 gyro){
        Rotation2d Rotation = ((BSGyro) gyro).getGyroscopeRotation();
        System.out.println(Rotation);
        BalanceDrive(gyro);
    }

    public void BalanceDrive(WPI_Pigeon2 gyro){
        if(gyro.getAngle() > 3){
            //DRIVE
            Translation2d test = new Translation2d(0,1).times(Constants.Swerve.maxSpeed / 8);
            Subsystems.swerveSubsystem.drive(test, 0, true, true);
        }
        if(gyro.getAngle() < 3){
            Translation2d test = new Translation2d(0,-1).times(Constants.Swerve.maxSpeed / 8);
            Subsystems.swerveSubsystem.drive(test, 0, true, true);
        }
        else{
            System.out.println("BALANCE SUB-SYS LOG: FLAT");
        }
    }

}