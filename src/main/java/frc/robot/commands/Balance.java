package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.PIDHelper;

public class Balance extends CommandBase implements Lifecycle {
    
    private final double pitchThreshold;
    private final static double BALANCE_TRANSLATION_SPEED = Constants.Swerve.maxSpeed / 8;


    private PIDController pidController = new PIDController(0.082, 0, 0.01);
    private PIDHelper pidHelper = new PIDHelper("BalancePID");


    public Balance() {
        this(3.0);
        addRequirements(Subsystems.swerveSubsystem);
        pidController.setSetpoint(0);
        pidController.setTolerance(0.5);

        pidHelper.initialize(0.1, 0, 0.01, 0, 0, 0);
    }

    public Balance(double pitchThreshold) {
        this.pitchThreshold = pitchThreshold;
    }

    @Override
    public void execute() {
        pidHelper.updateValuesFromDashboard();
        pidHelper.updatePIDController(pidController);

        final double currentPitch = Subsystems.swerveSubsystem.gyro.getPitch();

        double output = pidController.calculate(currentPitch);

        Translation2d translation = new Translation2d(output, 0).times(BALANCE_TRANSLATION_SPEED);
        Subsystems.swerveSubsystem.drive(translation, 0, true, true);
    }

}