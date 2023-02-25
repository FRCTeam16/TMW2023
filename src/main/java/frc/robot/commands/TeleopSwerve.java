package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    // Extended elevator constraings
    Constraints constraints = new Constraints(1.5, 1.5);
    TrapezoidProfile profile;

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

        // Govern speed when elevator is extended
        final double maxSpeed = Subsystems.elevator.isElevatorExtended() ? 1.5 : Constants.Swerve.maxSpeed;
        final double maxRotation = Subsystems.elevator.isElevatorExtended() ? 4 : Constants.Swerve.maxAngularVelocity;
        
        // if (Subsystems.elevator.isElevatorExtended()) {
        //     if (profile == null) {
        //         State goal = new TrapezoidProfile.State(maxSpeed, maxRotation)
        //                         profile = new TrapezoidProfile(constraints, null)
        //     }

        // }


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(maxSpeed), 
            rotationVal * maxRotation, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}