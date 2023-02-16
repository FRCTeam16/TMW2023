package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase implements Lifecycle {

    private TalonFX left = new TalonFX(Constants.Pivot.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Pivot.rightMotorId);

    private static double DEFAULT_OPENLOOP_SPEED = 0.25;

    private boolean openLoop = true;
    private double speed = 0.0;

    private double kFF=0.0015, kP=0.00012, kI=0, kD=0, maxVel=2000, maxAccel=500;
    private double setpoint = 0.0;



    public Pivot() {
        left.configFactoryDefault();
        right.configFactoryDefault();
        right.follow(left);
        right.setInverted(TalonFXInvertType.OpposeMaster);

        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        left.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        right.configAllSettings(config);
               

        this.zeroPivotEncoder();

        openLoop = true;
        speed = 0.0;

        left.config_kP(0, kP);
        left.config_kF(0, kFF);
        left.config_kI(0, kI);
        left.config_kD(0, kD);

        // motion magic
        left.configMotionCruiseVelocity(maxVel);
        left.configMotionAcceleration(maxAccel);


        SmartDashboard.setDefaultNumber("Pivot/kFF", kFF);
        SmartDashboard.setDefaultNumber("Pivot/kP", kP);
        SmartDashboard.setDefaultNumber("Pivot/kI", kI);
        SmartDashboard.setDefaultNumber("Pivot/kD", kD);
        SmartDashboard.setDefaultNumber("Pivot/MaxVel", maxVel);
        SmartDashboard.setDefaultNumber("Pivot/MaxAcc", maxAccel);

        SmartDashboard.setDefaultNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void zeroPivotEncoder() {
        this.left.setSelectedSensorPosition(0, 0, 20);
        this.right.setSelectedSensorPosition(0, 0, 20);
    }

    public void openLoopStop() {
        openLoop = true;
        speed = 0.0;
    }

    public void openLoopUp() {
        openLoop = true;
        speed = SmartDashboard.getNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void openLoopDown() {
        openLoop = true;
        speed = -SmartDashboard.getNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);
    }

    public void down() {
        openLoop = false;
        setpoint = 105.0;
    }

    public void up() {
        openLoop = false;
        setpoint = 0.0;
    }


    @Override
    public void periodic() {
        if (openLoop) {
            left.set(ControlMode.PercentOutput, speed);
        } else {
            double ff = SmartDashboard.getNumber("Pivot/kFF", kFF);
            double p = SmartDashboard.getNumber("Pivot/kP", kP);
            double i = SmartDashboard.getNumber("Pivot/kI", kI);
            double d = SmartDashboard.getNumber("Pivot/kD", kD);
            double v = SmartDashboard.getNumber("Pivot/MaxVel", maxVel);
            double a = SmartDashboard.getNumber("Pivot/MaxAcc", maxAccel);

            left.config_kP(0, p);
            left.config_kF(0, ff);
            left.config_kI(0, i);
            left.config_kD(0, d);

            // motion magic
            left.configMotionCruiseVelocity(v);
            left.configMotionAcceleration(a);

            left.set(ControlMode.MotionMagic, setpoint);
        }

        SmartDashboard.putBoolean("Pivot/OpenLoop", openLoop);
        SmartDashboard.putNumber("Pivot/Speed", speed);
        SmartDashboard.putNumber("Pivot/LeftEnc", left.getSelectedSensorPosition());
        SmartDashboard.putNumber("Pivot/LeftVel", left.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Pivot/BusV", left.getBusVoltage());
        SmartDashboard.putNumber("Pivot/OutAmp", left.getStatorCurrent());
        SmartDashboard.putString("Pivot/LastError", left.getLastError().toString());
    }
    
}
