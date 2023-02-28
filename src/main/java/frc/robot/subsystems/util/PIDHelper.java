package frc.robot.subsystems.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDHelper {
    private final String name;
    private final boolean useDashboard;

    public double kF = 0.0;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;

    public double kV = 0.0;
    public double kA = 0.0;

    public PIDHelper(String name) {
        this(name, true);
    }

    public PIDHelper(String name, boolean useDashboard) {
        this.name = name;
        this.useDashboard = useDashboard;
    }

    public void initialize(double kP, double kI, double kD, double kF, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kV = kV;
        this.kA = kA;

        if (this.useDashboard) {
            SmartDashboard.setDefaultNumber(name + "/kP", kP);
            SmartDashboard.setDefaultNumber(name + "/kI", kI);
            SmartDashboard.setDefaultNumber(name + "/kD", kD);
            SmartDashboard.setDefaultNumber(name + "/kF", kF);
            SmartDashboard.setDefaultNumber(name + "/kV", kV);
            SmartDashboard.setDefaultNumber(name + "/kA", kA);
        }
    }

    public void updateValuesFromDashboard() {
        if (!this.useDashboard) {
            // Do nothing if we don't use dashboard
            return;
        }
        double p = SmartDashboard.getNumber(name + "/kP", kP);
        double i = SmartDashboard.getNumber(name + "/kI", kI);
        double d = SmartDashboard.getNumber(name + "/kD", kD);
        double ff = SmartDashboard.getNumber(name + "/kF", kF);
        double v = SmartDashboard.getNumber(name + "/kV", kV);
        double a = SmartDashboard.getNumber(name + "/kA", kA);

        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = ff;
        this.kV = v;
        this.kA = a;
    }

    public void updateTalonFX(TalonFX motor, int slot) {
        motor.config_kP(slot, this.kP);
        motor.config_kI(slot, this.kI);
        motor.config_kD(slot, this.kD);
        motor.config_kF(slot, this.kF);
        motor.configMotionCruiseVelocity(this.kV);
        motor.configMotionAcceleration(this.kA);
    }

    public void updatePIDController(PIDController pid) {
        pid.setPID(this.kP, this.kI, this.kD);
    }

}
