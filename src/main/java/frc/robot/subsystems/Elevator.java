package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Lifecycle {

    private CANSparkMax left = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax right = new CANSparkMax(14, MotorType.kBrushless);

    private static double DEFAULT_OPENLOOP_SPEED = 0.25;

    private boolean openLoop = true;
    private double speed = 0.0;

    private double kFF=0.0015, kP=0.00012, kI=0, kD=0, maxVel=2000, maxAccel=500;
    private double setpoint = 0.0;



    public Elevator() {
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();
        right.follow(left, true);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        // Brownout protection to help avoid gate driver faults
        left.setOpenLoopRampRate(0.05); // 50 ms
        left.setSmartCurrentLimit(30);
        right.setOpenLoopRampRate(0.05);
        right.setSmartCurrentLimit(30);

        this.zeroElevatorEncoder();

        openLoop = true;
        speed = 0.0;

        
        // PID
        var pid = left.getPIDController();
        pid.setFF(kFF);
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setSmartMotionMaxVelocity(maxVel, 0);
        pid.setSmartMotionMaxAccel(maxAccel, 0);

        SmartDashboard.setDefaultNumber("Elevator/kFF", kFF);
        SmartDashboard.setDefaultNumber("Elevator/kP", kP);
        SmartDashboard.setDefaultNumber("Elevator/kI", kI);
        SmartDashboard.setDefaultNumber("Elevator/kD", kD);
        SmartDashboard.setDefaultNumber("Elevator/MaxVel", maxVel);
        SmartDashboard.setDefaultNumber("Elevator/MaxAcc", maxAccel);

        SmartDashboard.setDefaultNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void zeroElevatorEncoder() {
        this.left.getEncoder().setPosition(0.0);
    }

    public void stop() {
        openLoop = true;
        speed = 0.0;
    }

    public void forward() {
        openLoop = true;
        speed = SmartDashboard.getNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void reverse() {
        openLoop = true;
        speed = -SmartDashboard.getNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void extend() {
        openLoop = false;
        setpoint = 105.0;
    }

    public void retract() {
        openLoop = false;
        setpoint = 0.0;
    }


    @Override
    public void periodic() {
        if (openLoop) {
            left.set(speed);
        } else {
            double ff = SmartDashboard.getNumber("Elevator/kFF", kFF);
            double p = SmartDashboard.getNumber("Elevator/kP", kP);
            double i = SmartDashboard.getNumber("Elevator/kI", kI);
            double d = SmartDashboard.getNumber("Elevator/kD", kD);
            double v = SmartDashboard.getNumber("Elevator/MaxVel", maxVel);
            double a = SmartDashboard.getNumber("Elevator/MaxAcc", maxAccel);

            var pid = left.getPIDController();
            pid.setFF(ff);
            pid.setP(p);
            pid.setI(i);
            pid.setD(d);
            pid.setSmartMotionMaxVelocity(v, 0);
            pid.setSmartMotionMaxAccel(a, 0);
            pid.setReference(setpoint, ControlType.kSmartMotion);
        }

        SmartDashboard.putBoolean("Elevator/OpenLoop", openLoop);
        SmartDashboard.putNumber("Elevator/Speed", speed);
        SmartDashboard.putNumber("Elevator/LeftEnc", left.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator/LeftVel", left.getEncoder().getVelocity());

        SmartDashboard.putNumber("Eleveator/BusV", left.getBusVoltage());
        SmartDashboard.putNumber("Elevator/OutAmp", left.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/StickyFault", left.getStickyFaults());
    }
    
}
