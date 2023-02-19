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
import frc.robot.subsystems.util.PIDHelper;


public class Intake extends SubsystemBase implements Lifecycle {

    private Solenoid upper = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private Solenoid lower = new Solenoid(PneumaticsModuleType.REVPH, 1);
    private boolean Solstend = false;

    private TalonFX left = new TalonFX(Constants.Intake.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Intake.rightMotorId);
    private TalonFX wrist = new TalonFX(Constants.Intake.wristMotorId);

    private static final double DEFAULT_OPENLOOP_WRIST_SPEED = 0.25;
    private static final double DEFAULT_OPENLOOP_INTAKE_SPEED = 0.5;

    private boolean openLoop = true;
    private double openLoopWristSpeed = 0.0;

    private double intakeSpeed = 0.0;

    private PIDHelper pidHelper = new PIDHelper("Intake");
    private double setpoint = 0.0;


    public enum WristPosition {
        Vertical(0),
        GroundPickup(45207);

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

        pidHelper.initialize(0.00012, 0, 0, 0, 0, 0);
        pidHelper.updateTalonFX(left, 0);

        SmartDashboard.setDefaultNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_WRIST_SPEED);
        SmartDashboard.setDefaultNumber("Intake/IntakeSpeed", DEFAULT_OPENLOOP_INTAKE_SPEED);

        left.configForwardSoftLimitThreshold(60000);
        left.configReverseSoftLimitThreshold(0);
    }

    @Override
    public void teleopInit() {
        setpoint = left.getSelectedSensorPosition();
    }

    public void zeroWristEncoder() {
        this.wrist.setSelectedSensorPosition(0, 0, 20);
        this.setpoint = 0.0;
    }

    public void stopWrist() {
        openLoop = true;
        openLoopWristSpeed = 0.0;
    }

    public void raiseWristOpenLoop() {
        openLoop = true;
        openLoopWristSpeed = SmartDashboard.getNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_WRIST_SPEED);
    }

    public void lowerWristOpenLoop() {
        openLoop = true;
        openLoopWristSpeed = -SmartDashboard.getNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_WRIST_SPEED);
    }

    public void intake() {
        intakeSpeed = SmartDashboard.getNumber("Intake/IntakeSpeed", DEFAULT_OPENLOOP_INTAKE_SPEED);
    }

    public void eject() {
        intakeSpeed = -SmartDashboard.getNumber("Intake/IntakeSpeed", DEFAULT_OPENLOOP_INTAKE_SPEED);
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

    public void setSoftLimitsEnabled(boolean enable) {
        left.configForwardSoftLimitEnable(enable);
        left.configReverseSoftLimitEnable(enable);
    }

    @Override
    public void periodic() {
        // Always run intake in open loop
        left.set(ControlMode.PercentOutput, intakeSpeed);

        // Wrist can run in open loop or closed loop control
        if (openLoop) {
            wrist.set(ControlMode.PercentOutput, openLoopWristSpeed);
        } else {
            pidHelper.updateValuesFromDashboard();
            pidHelper.updateTalonFX(left, 0);
           
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
        lower.set(!Solstend);
    }
    // solinoid is all I think about
    public void OpenHand(){
        System.out.println("OpenHand Called");
        Solstend = true;
    }

    public void CloseHand(){
        System.out.println("CloseHand Called");
        Solstend = false;
    }
    
}
