package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.PartIndicator.PartType;
import frc.robot.subsystems.util.PIDHelper;

public class Intake extends SubsystemBase implements Lifecycle {

    private Solenoid upper = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private Solenoid lower = new Solenoid(PneumaticsModuleType.REVPH, 1);
    private DoubleSolenoid puncher = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
    private boolean Solstend = false;
    private boolean solenoidStateChanged = false;
    private boolean puncherStateChanged = false;
    private DoubleSolenoid.Value punchExtend = Value.kReverse;
    private boolean launchStart = false;
    private int launchDelayCounter = 0;

    // private Solenoid extraSol1 = new Solenoid(PneumaticsModuleType.REVPH, 2);
    // private Solenoid extraSol2 = new Solenoid(PneumaticsModuleType.REVPH, 3);

    // private Solenoid extraSol3 = new Solenoid(PneumaticsModuleType.REVPH, 4);
    // private Solenoid extraSol4 = new Solenoid(PneumaticsModuleType.REVPH, 5);

    // private Solenoid extraSol5 = new Solenoid(PneumaticsModuleType.REVPH, 6);
    // private Solenoid extraSol6 = new Solenoid(PneumaticsModuleType.REVPH, 7);

    // private Solenoid extraSol7 = new Solenoid(PneumaticsModuleType.REVPH, 8);
    // private Solenoid extraSol8 = new Solenoid(PneumaticsModuleType.REVPH, 9);

    private TalonFX left = new TalonFX(Constants.Intake.leftMotorId);
    private TalonFX right = new TalonFX(Constants.Intake.rightMotorId);
    private TalonFX wrist = new TalonFX(Constants.Intake.wristMotorId);

    private static final double DEFAULT_OPENLOOP_WRIST_SPEED = 0.25;
    private static final double DEFAULT_OPENLOOP_INTAKE_SPEED = 0.50;
    private static final double DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED = 0.05;
    private static final double DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED = 0.13;

    private boolean openLoop = true;
    private double openLoopWristSpeed = 0.0;

    private double intakeSpeed = 0.0;
    private double slowIntakeSpeed = DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED;

    private PIDHelper pidHelper = new PIDHelper("Intake", false);
    private double setpoint = 0.0;
    private double storedSetpoint = 0.0;

    private DigitalInput ProxSence = new DigitalInput(0); // Digital input might be 9 but we'll go w/ 0 for now

    public enum IntakeConditions {
        Stop(0),
        Eject(1),
        Intake(2),
        Launch(3),
        Hold(4);
    
        public final int setIntakeSate;

        private IntakeConditions(int setIntakeSate) {
            this.setIntakeSate = setIntakeSate;
        }
    }

    private IntakeConditions IntakeState = IntakeConditions.Stop;

    boolean hasPart = false;

    public enum WristPosition {
        Vertical(0),
        SingleSubstation(-89_500),
        DoubleSubstation(-127_540),
        ScoreCone(1000),
        ScoreCube(1000),
        GroundPickup(30_850),
        GroundPickupCube(-35124),
        Stow(-63_467),
        ScoreCubeHigh(-150_383),
        ScoreCubeMid(-150_242),
        WristScore(-75_843),
        Shooter(-577889); // GET VALS FOR THIS

        public final double setpoint;

        private WristPosition(double setpoint) {
            this.setpoint = setpoint;
        }



    }

    public enum HandState {
        Open(1),
        Closed(0);

        public final int setState;

        private HandState(int setState) {
            this.setState = setState;
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
        hasPart = false;
        ClosePuncher();
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
        IntakeState = IntakeConditions.Intake;
    }

    public void eject() {
        intakeSpeed = -SmartDashboard.getNumber("Intake/IntakeEjectSpeed", DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED);
        IntakeState = IntakeConditions.Eject;    }

    public void runSlowIntake() {
        intakeSpeed = SmartDashboard.getNumber("Intake/SlowIntakeSpeed", DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED);
        IntakeState = IntakeConditions.Hold;
    }

    public void setAtSubstation(boolean atSubstation) {
        if(atSubstation && !isProxTripped()) {
            intake();
        }
        else {
            if (!DriverStation.isAutonomous()) {
                stopIntake();
            }
        }
    }

    public void stopIntake() {
        intakeSpeed = 0.0;
        IntakeState = IntakeConditions.Stop;
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
        slowIntakeSpeed = SmartDashboard.getNumber("Intake/SlowIntakeSpeed", DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED);

        if (!DriverStation.isAutonomousEnabled()) {
            // Handle hasPart handling
            if (isProxTripped() && !hasPart) {
                hasPart = true;

                // If we are not in Substation and picking up a cube then automatically close
                // the hand
                if (Subsystems.poseManager.getCurrentPose() != Pose.SingleSubstation &&
                    Subsystems.poseManager.getCurrentPose() != Pose.GroundPickup &&
                        Subsystems.partIndicator.requestedPartType == PartType.Cone) {
                    CloseHand();
                    }
                }

                // We had a part, signal we no longer have it
                if (!isProxTripped() && hasPart) {
                    hasPart = false;
                    Subsystems.partIndicator.requestPart(PartType.None);
                }

                if (IntakeState == IntakeConditions.Stop && hasPart) {
                    IntakeState = IntakeConditions.Hold;
            }

        switch (IntakeState) {
            case Stop: left.set(ControlMode.PercentOutput, 0);
                break;
            case Eject: left.set(ControlMode.PercentOutput, -SmartDashboard.getNumber("Intake/IntakeEjectSpeed", DEFAULT_OPENLOOP_INTAKE_EJECT_SPEED));
                break;
            case Intake: left.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Intake/IntakeSpeed", DEFAULT_OPENLOOP_INTAKE_SPEED));
                break;
            case Hold: left.set(ControlMode.PercentOutput, SmartDashboard.getNumber("Intake/SlowIntakeSpeed", DEFAULT_OPENLOOP_SLOW_INTAKE_SPEED));
                break;
            case Launch: left.set(ControlMode.PercentOutput, -1.0);
            }

        } else {
            // Auto keeps running
            left.set(ControlMode.PercentOutput, intakeSpeed);
       }

       if (launchStart) {
            if(launchDelayCounter < 12) {
                launchDelayCounter++;
            }
            else {
                IntakeState = IntakeConditions.Launch;   
            }
       }
       else {
        launchDelayCounter = 0;
       }

       
 
       /*   ***************Removed for code above... but i'm not totally confident in it.  :)
        if (!DriverStation.isAutonomousEnabled()) {
            // Handle hasPart handling
            if (isProxTripped() && !hasPart) {
                hasPart = true;
                intakeSpeed = 0.0; // hack for speed control?

                // If we are not in Substation and picking up a cube then automatically close
                // the hand
                if (Subsystems.poseManager.getCurrentPose() != Pose.SingleSubstation &&
                        Subsystems.partIndicator.requestedPartType == PartType.Cone) {
                    CloseHand();
                }
            }


            // Intake Speed Control
            // Always run intake in open loop
            if (isProxTripped() && Math.abs(intakeSpeed) < 0.01) {
                left.set(ControlMode.PercentOutput, slowIntakeSpeed);
            } else {

                if (hasPart && Subsystems.poseManager.getCurrentPose() == Pose.SingleSubstation) {
                    left.set(ControlMode.PercentOutput, slowIntakeSpeed);
                } else {
                    left.set(ControlMode.PercentOutput, intakeSpeed);
                }

                // We had a part, signal we no longer have it
                if (!isProxTripped() && hasPart) {
                    hasPart = false;
                    Subsystems.partIndicator.requestPart(PartType.None);
                }
            }
        } else {
            // Auto keeps running
            left.set(ControlMode.PercentOutput, intakeSpeed);
        }

        
*/

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

        if (solenoidStateChanged) {
            upper.set(Solstend); // open/close solinoids
            lower.set(!Solstend);
            solenoidStateChanged = false;
        }

        //need to set logic for punching cube/cone
        //puncher.set(DoubleSolenoid.Value.kForward);
        if (puncherStateChanged){
            puncher.set(punchExtend);
            puncherStateChanged = false;
        }

    }

    // solinoid is all I think about
    public void OpenHand(){
        solenoidStateChanged = true;
        Solstend = false;
    }

    public void CloseHand(){
        solenoidStateChanged = true;
        Solstend = true;
    }

    public boolean isHandOpen() {
        return !Solstend;
    }

    public boolean isProxTripped(){
        return !ProxSence.get();
    }

    public boolean hasPart() {
        return this.hasPart;
    }

    //puncher
    public void OpenPuncher(){
        puncherStateChanged = true;
        punchExtend = Value.kReverse;
        //CloseHand();
        launchStart = true;
    }

    public void ClosePuncher(){
        puncherStateChanged = true;
        punchExtend = Value.kForward;
        IntakeState = IntakeConditions.Stop;
        launchStart = false;
    }
}
