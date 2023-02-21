package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class RotationController extends PIDController {

    private static final double maxDegreesPerSecond = Math.toDegrees(Constants.Swerve.maxAngularVelocity);

    private static final double kP = 4.0; //0.01; //4.25;
    private static final double kI = 1.35; // 3.0;
    private static final double kD = 0.0; // 0;

    private double tolerance = 2.0;

    public RotationController() {
        this(kP, kI, kD);
    }

    public RotationController(double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.enableContinuousInput(-180, 180);
        this.setIntegratorRange(-5, 5);
        this.setTolerance(tolerance);
    }

    public static double clampToDPS(double outputPercent) {
        return MathUtil.clamp(outputPercent, -0.6, 0.6) * maxDegreesPerSecond;
    }
}
