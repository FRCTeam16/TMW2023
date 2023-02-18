package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.util.PIDHelper;

public class Pivot extends SubsystemBase implements Lifecycle {

    private TalonFX left = new TalonFX(Constants.Pivot.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Pivot.rightMotorId);

    private static double DEFAULT_OPENLOOP_SPEED = 0.5;

    private boolean openLoop = true;
    private double speed = 0.0;

    private final PIDHelper pidHelper = new PIDHelper("Pivot");
    private double setpoint = 0.0;


    public enum PivotPosition {
        Vertical(0),
        Horizontal(200000),
        GroundPickup(383480);

        public final double setpoint;

        private PivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }


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
               
        pidHelper.initialize(0.026, 0, 0, 0, 0, 0);
        pidHelper.updateTalonFX(left, 0);
        
        SmartDashboard.setDefaultNumber("Pivot/OpenLoopSpeedx", DEFAULT_OPENLOOP_SPEED);

        this.zeroPivotEncoder();
        openLoop = true;
        speed = 0.0;
    }

    public void zeroPivotEncoder() {
        this.left.setSelectedSensorPosition(0, 0, 20);
        this.right.setSelectedSensorPosition(0, 0, 20);
        this.setpoint = 0.0;
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

    public void holdPosition() {
        this.setPivotSetpoint(left.getSelectedSensorPosition());
    }

    public void setPivotPosition(PivotPosition position) {
        this.setPivotSetpoint(position.setpoint);
    }

    private void setPivotSetpoint(double setpoint) {
        openLoop = false;
        this.setpoint = setpoint;
    }

    /**
     * Returns the angle of the pivoted arm, with 0 degrees at the top
     * @return
     */
    public double getPivotAngleDegrees() {
        int kMeasuredPosHorizontal = -311371; // FIXME: Position measured when arm is horizontal
        double gearing = 625;
        double kTicksPerDegree = (gearing * 4096) / 360; //Sensor is 1:1 with arm rotation

        kTicksPerDegree = 311371 / 90;
        double currentPos = left.getSelectedSensorPosition();
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        return degrees - 90.0;
    }

    @Override
    public void periodic() {
        if (openLoop) {
            left.set(ControlMode.PercentOutput, speed);
        } else {
            pidHelper.updateValuesFromDashboard();
            pidHelper.updateTalonFX(left, 0);

            // left.set(ControlMode.MotionMagic, setpoint);
            left.set(ControlMode.Position, setpoint);
            // left.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, calculateGravityFeedForward() );
        }

        SmartDashboard.putBoolean("Pivot/OpenLoop", openLoop);
        SmartDashboard.putNumber("Pivot/Speed", speed);
        SmartDashboard.putNumber("Pivot/LeftEnc", left.getSelectedSensorPosition());
        SmartDashboard.putNumber("Pivot/LeftVel", left.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Pivot/setpoint", this.setpoint);
        SmartDashboard.putNumber("Pivot/Degrees", this.getPivotAngleDegrees());

        SmartDashboard.putNumber("Pivot/BusV", left.getBusVoltage());
        SmartDashboard.putNumber("Pivot/OutAmp", left.getStatorCurrent());
        SmartDashboard.putString("Pivot/LastError", left.getLastError().toString());
    }
    
}
