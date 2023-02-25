package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Swerve;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    // Extended elevator constraints
    private static final double MAX_TRANSLATION_SPEED = 1.5;    // m/s
    private static final double DECEL_RATE = 0.07;

    private Translation2d lastSpeed = new Translation2d(0,0);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // Define maximum speeds
        double maxSpeed = Subsystems.elevator.isElevatorExtended() ? MAX_TRANSLATION_SPEED : Constants.Swerve.maxSpeed;
        final double maxRotation = Subsystems.elevator.isElevatorExtended() ? 4 : Constants.Swerve.maxAngularVelocity;

        // Target swerve velocity state
        final Translation2d swerveSpeed;
        Translation2d targetSpeed = new Translation2d(translationVal, strafeVal).times(maxSpeed);
        if (Subsystems.elevator.isElevatorExtended() && 
            (lastSpeed.getX() >= targetSpeed.getX() || lastSpeed.getY() >= targetSpeed.getY())) {
                // Interpolate slowly towards target if we are extended and going faster than target velocity
            swerveSpeed = new Translation2d(
                MathUtil.interpolate(lastSpeed.getX(), targetSpeed.getX(), DECEL_RATE),
                MathUtil.interpolate(lastSpeed.getY(), targetSpeed.getY(), DECEL_RATE)
            );
        } else {
            swerveSpeed = targetSpeed;
        }

        lastSpeed = swerveSpeed;

        /* Drive */
        s_Swerve.drive(
            swerveSpeed, 
            rotationVal * maxRotation, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}