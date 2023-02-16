package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase implements Lifecycle {

    private Solenoid upper = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private Solenoid lower = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    private Boolean Solstend = false;

    private TalonFX left = new TalonFX(Constants.Intake.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Intake.rightMotorId);
    private TalonFX wrist = new TalonFX(Constants.Intake.wristMotorId);

    private static double DEFAULT_OPENLOOP_SPEED = 0.25;

    private boolean openLoop = true;
    private double openLoopWristSpeed = 0.0;

    private double intakeSpeed = 0.0;

    private double kFF=0.00, kP=0.00012, kI=0, kD=0, maxVel=2000, maxAccel=500;
    private double setpoint = 0.0;


    public enum WristPosition {
        Vertical(0),
        Down(100);

        public final double setpoint;

        private WristPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }



    public Intake() {
        left.configFactoryDefault();
        right.configFactoryDefault();
        wrist.configFactoryDefault();
        right.follow(left);
        right.setInverted(TalonFXInvertType.OpposeMaster);

        left.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
        wrist.setNeutralMode(NeutralMode.Brake);


        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
        config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        left.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        right.configAllSettings(config);
        wrist.configAllSettings(config);
               
        this.zeroWristEncoder();

        openLoop = true;
        openLoopWristSpeed = 0.0;

        wrist.config_kP(0, kP);
        wrist.config_kF(0, kFF);
        wrist.config_kI(0, kI);
        wrist.config_kD(0, kD);

        // motion magic
        wrist.configMotionCruiseVelocity(maxVel);
        wrist.configMotionAcceleration(maxAccel);


        SmartDashboard.setDefaultNumber("Intake/kFF", kFF);
        SmartDashboard.setDefaultNumber("Intake/kP", kP);
        SmartDashboard.setDefaultNumber("Intake/kI", kI);
        SmartDashboard.setDefaultNumber("Intake/kD", kD);
        SmartDashboard.setDefaultNumber("Intake/MaxVel", maxVel);
        SmartDashboard.setDefaultNumber("Intake/MaxAcc", maxAccel);

        SmartDashboard.setDefaultNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_SPEED);
        SmartDashboard.setDefaultNumber("Intake/IntakeSpeed", 0.1);
    }

    public void zeroWristEncoder() {
        this.wrist.setSelectedSensorPosition(0, 0, 20);
    }

    public void stopWrist() {
        openLoop = true;
        openLoopWristSpeed = 0.0;
    }

    public void raiseWristOpenLoop() {
        openLoop = true;
        openLoopWristSpeed = SmartDashboard.getNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void lowerWristOpenLoop() {
        openLoop = true;
        openLoopWristSpeed = -SmartDashboard.getNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_SPEED);
    }

    public void intake() {
        intakeSpeed = SmartDashboard.getNumber("Intake/IntakeSpeed", 0.25);
    }

    public void eject() {
        intakeSpeed = -SmartDashboard.getNumber("Intake/IntakeSpeed", 0.25);
    }

    public void stopIntake() {
        intakeSpeed = 0.0;
    }

    public void holdWrist() {
        double currentPosition = wrist.getSelectedSensorPosition();
        setWristSetpoint(currentPosition);
    }

    public void setWristPosition(WristPosition position) {
        setWristSetpoint(position.setpoint);
    }

    private void setWristSetpoint(double setpoint) {
        openLoop = false;
        this.setpoint = setpoint;
    }


    @Override
    public void periodic() {
        // Always run intake in open loop
        left.set(ControlMode.PercentOutput, intakeSpeed);

        // Wrist can run in open loop or closed loop control
        if (openLoop) {
            wrist.set(ControlMode.PercentOutput, openLoopWristSpeed);
        } else {
            double ff = SmartDashboard.getNumber("Intake/kFF", kFF);
            double p = SmartDashboard.getNumber("Intake/kP", kP);
            double i = SmartDashboard.getNumber("Intake/kI", kI);
            double d = SmartDashboard.getNumber("Intake/kD", kD);
            double v = SmartDashboard.getNumber("Intake/MaxVel", maxVel);
            double a = SmartDashboard.getNumber("Intake/MaxAcc", maxAccel);

            wrist.config_kP(0, p);
            wrist.config_kF(0, ff);
            wrist.config_kI(0, i);
            wrist.config_kD(0, d);

            // motion magic
            wrist.configMotionCruiseVelocity(v);
            wrist.configMotionAcceleration(a);

            // wrist.set(ControlMode.MotionMagic, setpoint);
            wrist.set(ControlMode.Position, setpoint);
        }

        SmartDashboard.putBoolean("Intake/OpenLoop", openLoop);
        SmartDashboard.putNumber("Intake/wristEnc", wrist.getSelectedSensorPosition());
        SmartDashboard.putNumber("Intake/wristVel", wrist.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Intake/wristSetpoint", setpoint);

        SmartDashboard.putNumber("Intake/BusV", wrist.getBusVoltage());
        SmartDashboard.putNumber("Intake/OutAmp", wrist.getStatorCurrent());
        SmartDashboard.putString("Intake/LastError", wrist.getLastError().toString());

        upper.set(Solstend); //open/close solinoids
        lower.set(Solstend);
    }
    // solinoid is all I think about
    void OpenHand(){
        Solstend = true;
    }

    void CloseHand(){
        Solstend = false;
    }
    
}
