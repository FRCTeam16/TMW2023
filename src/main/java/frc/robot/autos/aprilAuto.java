package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;


public class aprilAuto extends CommandBase{

    double Rx;
    double Ry;


    private class NetTable {
        double tx;
        double ty;
        double ID;
    }

    int checkNetT(NetTable NetT){
        if(NetT.tx > 0){
            return 1;
        }
        if(NetT.tx < 0){
            return -1;
        }
        return 0;
    }

    @Override
    public void initialize() {
        System.out.println("April Auto Init");
    }

    @Override
    public void execute(){
        NetTable instance = newNetT();
        //TODO confirm that it is corrected propperly
        AprilDrive(instance);
    }

    @Override
    public boolean isFinished(){
        NetTable instance = newNetT();
        if(instance.tx < 1 && instance.tx > -1) { //check if in deadzone threshold
            return true;
        }
        return false;
    }

    public void AprilDrive(NetTable instance){
        if( instance.ID == 1 ||
            instance.ID == 2 ||
            instance.ID == 3 ||
            instance.ID == 4 ){
            //drive in pos way
            Translation2d translation = new Translation2d(Ry ,checkNetT(instance) + Rx ).times(Constants.Swerve.maxSpeed / 6); //the 8 can be reduced to go faster
            Subsystems.swerveSubsystem.drive(translation, 0, true, true);
        }
        if( instance.ID == 5 ||
            instance.ID == 6 ||
            instance.ID == 7 ||
            instance.ID == 8 ){
            //drive in neg way
            Translation2d translation = new Translation2d(Ry ,-checkNetT(instance) + Rx ).times(Constants.Swerve.maxSpeed / 6); //the 8 can be reduced to go faster
            Subsystems.swerveSubsystem.drive(translation, 0, true, true);
        }
        else{
            System.out.println("APRIL AUTO- UNREGISTERED ID");
        }
    }

    //makes a new net table
    public NetTable newNetT(){
        NetTable NewT = new NetTable();
        NewT.tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        NewT.ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        NewT.ID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("fid").getDouble(0);
        return NewT;
    }

    public aprilAuto(Double Rx, Double Ry){
        System.out.println("APRIL AUTO INIT");
        addRequirements(Subsystems.swerveSubsystem); //so presets dont fight
    }
}