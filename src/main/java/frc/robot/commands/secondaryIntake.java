package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.PIDHelper;


// UNFINISHED CODE FOR UNFINISHED INTAKE, TODO: ADD NEO AND SPARK MAX SUFF

public class secondaryIntake {

    private PIDController pidController = new PIDController(0, 0, 0);
    private PIDHelper pidHelper = new PIDHelper("siPID");

    double TMPSetpoint = 15000; //DONT trust this, arbitrary number

    public secondaryIntake(){
        pidController.setSetpoint(TMPSetpoint);
        pidController.setTolerance(0.5);
    
        pidHelper.initialize(0.08, 0, 0.01, 0, 0, 0);
    }

    public void siDrive(double input){
        //funtion to move Motor by x value
        // input is current value while [ TMPSetpoint ] is set value

        double sidCalc = pidController.calculate(input);


    }
}
