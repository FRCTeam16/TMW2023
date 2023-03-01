package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.PartIndicator.PartType;
import frc.robot.subsystems.util.PIDHelper;

public class Intake extends SubsystemBase implements Lifecycle {

    private Solenoid upper = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private Solenoid lower = new Solenoid(PneumaticsModuleType.REVPH, 1);
    private boolean Solstend = false;

    private TalonFX left = new TalonFX(Constants.Intake.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Intake.rightMotorId);
    private TalonFX wrist = new TalonFX(Constants.Intake.wristMotorId);

    private static final double DEFAULT_OPENLOOP_WRIST_SPEED = 0.25;
    private static final double DEFAULT_OPENLOOP_INTAKE_SPEED = 0.25;
    private static final double DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED = 0.05;
    private static final double DEFAULT_OPENLOOP_TRAVELINTAKE_SPEED = 0.05;
    private static final double DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED = 0.15;

    private boolean openLoop = true;
    private double openLoopWristSpeed = 0.0;

    private double intakeSpeed = 0.0;
    private double slowIntakeSpeed = DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED;

    private PIDHelper pidHelper = new PIDHelper("Intake");
    private double setpoint = 0.0;
    private double storedSetpoint = 0.0;

    private DigitalInput ProxSence = new DigitalInput(0); // Digital input might be 9 but we'll go w/ 0 for now

    public enum WristPosition {
        Vertical(0),
        SingleSubstation(-89_500),
        DoubleSubstation(-127_540),
        ScoreCone(1000), // 50000
        ScoreCube(1000), // 100000
        GroundPickup(30_850), // was -45207
        Stow(-72_488),
        ScoreCubeHigh(-150_383),
        ScoreCubeMid(-150_242),
        WristScore(-75_843);

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

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
        config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit
                                                           // triggers, in sec
        config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
        config.neutralDeadband = 0.001;
        
        left.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
        right.configAllSettings(config);
        wrist.configAllSettings(config);

        // Custom settings
        left.setNeutralMode(NeutralMode.Coast);
        right.setNeutralMode(NeutralMode.Coast);
        wrist.setNeutralMode(NeutralMode.Brake);

        this.zeroWristEncoder();

        openLoop = true;
        openLoopWristSpeed = 0.0;

        pidHelper.initialize(0.07, 0, 0, 0, 0, 0);
        pidHelper.updateTalonFX(wrist, 0);

        SmartDashboard.setDefaultNumber("Intake/OpenLoopWristSpeed", DEFAULT_OPENLOOP_WRIST_SPEED);
        SmartDashboard.setDefaultNumber("Intake/IntakeSpeed", DEFAULT_OPENLOOP_INTAKE_SPEED);
        SmartDashboard.setDefaultNumber("Intake/SlowIntakeSpeed", DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED);
        SmartDashboard.setDefaultNumber("Intake/TravelIntakeSpeed", DEFAULT_OPENLOOP_TRAVELINTAKE_SPEED);
        SmartDashboard.setDefaultNumber("Intake/IntakeEjectSpeed", DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED);

        wrist.configForwardSoftLimitThreshold(40_000);
        wrist.configReverseSoftLimitThreshold(-170_000);
        this.setSoftLimitsEnabled(true);

        // Start in closed
        this.CloseHand();

        this.slowIntakeSpeed = SmartDashboard.getNumber("Intake/SlowIntakeSpeed", DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED);
    }

    @Override
    public void teleopInit() {
        setpoint = wrist.getSelectedSensorPosition();
    }

    @Override
    public void autoInit() {
        
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
        intakeSpeed = -SmartDashboard.getNumber("Intake/IntakeEjectSpeed", DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED);
    }

    public void runTravelIntake() {
        intakeSpeed = SmartDashboard.getNumber("Intake/TravelIntakeSpeed", DEFAULT_OPENLOOP_TRAVELINTAKE_SPEED);
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

    public void storeAndScore() {
        storedSetpoint = setpoint;
        setWristSetpoint(WristPosition.WristScore.setpoint);
    }

    public void restoreStoredSetpoint() {
        setWristSetpoint(storedSetpoint);
    }

    private void setWristSetpoint(double setpoint) {
        openLoop = false;
        this.setpoint = setpoint;
    }

    public void setSoftLimitsEnabled(boolean enable) {
        wrist.configForwardSoftLimitEnable(enable);
        wrist.configReverseSoftLimitEnable(enable);
    }

    @Override
    public void periodic() {

        if(isProxTripped()){
            // Always run intake in open loop
            left.set(ControlMode.PercentOutput, slowIntakeSpeed);
        } else{
            Subsystems.partIndicator.requestPart(PartType.None);    // call out signal
            left.set(ControlMode.PercentOutput, intakeSpeed);
        }

        // Wrist can run in open loop or closed loop control
        if (openLoop) {
            wrist.set(ControlMode.PercentOutput, openLoopWristSpeed);
        } else {
            pidHelper.updateValuesFromDashboard();
            pidHelper.updateTalonFX(wrist, 0);

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
        SmartDashboard.putBoolean("Intake/ObjectDetected", this.isProxTripped());

        upper.set(Solstend); // open/close solinoids
        lower.set(!Solstend);
    }

    // solinoid is all I think about
    public void OpenHand(){
        Solstend = false;
    }

    public void CloseHand(){
        Solstend = true;
    }

    public boolean isHandOpen() {
        return !Solstend;
    }

    public boolean isProxTripped(){
        return !ProxSence.get();
    }
}
