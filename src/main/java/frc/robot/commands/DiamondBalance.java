package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.util.PIDHelper;

public class DiamondBalance extends CommandBase implements Lifecycle{

    private PIDHelper pidHelper = new PIDHelper("DiamondBalancePID");

    public DiamondBalance(){
        addRequirements(Subsystems.swerveSubsystem);
        //double CAngle = SmartDashboard.getNumber("", 0);

    }

    @Override
    public void execute(){
        //get latest values
        //double CAngle = 

       
    }

}
