package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.util.PIDHelper;

public class Elevator extends SubsystemBase implements Lifecycle {

    private TalonFX left = new TalonFX(Constants.Elevator.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Elevator.rightMotorId);

    private static double DEFAULT_OPENLOOP_SPEED = 0.15;

    private boolean openLoop = true;
    private double speed = 0.0;

    private final PIDHelper pidHelper = new PIDHelper("Elevator");
    private double setpoint = 0.0;


    public enum ElevatorPosition {
        StartPosition(0),
        Retracted(100),
        LowConePlacement(200),
        HightConePlacement(500);

        public final double setpoint;

        private ElevatorPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public Elevator() {
        left.configFactoryDefault();
        right.configFactoryDefault();
        right.follow(left);

        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        left.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        right.configAllSettings(config);
               
        pidHelper.initialize(0.00012, 0, 0, 0, 0, 0);
        pidHelper.updateTalonFX(left, 0);

        SmartDashboard.setDefaultNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);

        this.zeroElevatorEncoder();
        openLoop = true;
        speed = 0.0;
    }

    public void zeroElevatorEncoder() {
        this.left.setSelectedSensorPosition(0, 0, 20);
        this.right.setSelectedSensorPosition(0, 0, 20);
    }

    public void stop() {
        speed = 0.0;
        this.setElevatorSetpoint(this.left.getSelectedSensorPosition());
    }

    public void forward() {
        openLoop = true;
        speed = SmartDashboard.getNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void reverse() {
        openLoop = true;
        speed = -SmartDashboard.getNumber("Elevator/OpenLoopSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void setElevatorPosition(ElevatorPosition position) {
        this.setElevatorSetpoint(position.setpoint);
    }

    private void setElevatorSetpoint(double setpoint) {
        openLoop = false;
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        if (openLoop) {
            left.set(ControlMode.PercentOutput, speed);
        } else {
            pidHelper.updateValuesFromDashboard();
            pidHelper.updateTalonFX(left, 0);

            left.set(ControlMode.Position, setpoint);
            // left.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, calculateGravityFeedForward() );
            // left.set(ControlMode.MotionMagic, setpoint);  TODO: measure velocity and acceleration for motion magic
        }

        // Telemetry
        SmartDashboard.putBoolean("Elevator/OpenLoop", openLoop);
        SmartDashboard.putNumber("Elevator/Speed", speed);
        SmartDashboard.putNumber("Elevator/LeftEnc", left.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator/LeftVel", left.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Elevator/setpoint", this.setpoint);

        SmartDashboard.putNumber("Elevator/BusV", left.getBusVoltage());
        SmartDashboard.putNumber("Elevator/OutAmp", left.getStatorCurrent());
        SmartDashboard.putString("Elevator/LastError", left.getLastError().toString());
    }

    /**
     * Calculates an arbitrary feed forward value to offset gravity effects
     * 
     * @see https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#gravity-offset-arm
     */
    private double calculateGravityFeedForward() {
        double degrees = Subsystems.pivot.getPivotAngleDegrees();
        double radians = java.lang.Math.toRadians(degrees);
        double cosineScalar = java.lang.Math.cos(radians);
        double maxGravityFF = 0.07;
        return maxGravityFF * cosineScalar;
    }
    
}
