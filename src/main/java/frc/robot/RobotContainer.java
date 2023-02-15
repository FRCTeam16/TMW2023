package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.auto.AutoManager;
import frc.robot.autos.aprilAuto;
import frc.robot.autos.boto;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.TeleopSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick left = new Joystick(0);
    private final Joystick right = new Joystick(1);
    private final Joystick gamepad = new Joystick(2);


    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(gamepad, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(right, 2);

    private final JoystickButton elevatorForward = new JoystickButton(gamepad, XboxController.Button.kY.value);
    private final JoystickButton elevatorReverse = new JoystickButton(gamepad, XboxController.Button.kX.value);

    private final JoystickButton extendRamp = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);
    private final JoystickButton retractRamp = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);

    private final JoystickButton april = new JoystickButton(right, 4);
    private final JoystickButton boto = new JoystickButton(right, 3);


    /* Subsystems */
    private final Subsystems subsystems = Subsystems.getInstance();
    private final AutoManager autoManager = new AutoManager();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Subsystems.swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                Subsystems.swerveSubsystem, 
                () -> -right.getY(),
                () -> -right.getX(), 
                () -> left.getX(), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Configure software buttons
        configureDashboardButtons();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> Subsystems.swerveSubsystem.zeroGyro()));

        april.onTrue(new aprilAuto(right.getX(),right.getY()));
        boto.onTrue(new boto(right.getRawAxis(0),right.getRawAxis(1)));

        elevatorForward
            .onTrue(new InstantCommand(() -> Subsystems.elevator.forward()))
            .onFalse(new InstantCommand(() -> Subsystems.elevator.stop()));
        elevatorReverse
            .onTrue(new InstantCommand(() -> Subsystems.elevator.reverse()))
            .onFalse(new InstantCommand(() -> Subsystems.elevator.stop()));

        extendRamp.onTrue(new InstantCommand(() -> Subsystems.ramp.forward()));
        retractRamp.onTrue(new InstantCommand(() -> Subsystems.ramp.back()));
    }


    private void configureDashboardButtons() {
        SmartDashboard.putData("Zero Elevator Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.elevator.zeroElevatorEncoder()));
        SmartDashboard.putData("Retract Elevator", new InstantCommand(() -> Subsystems.elevator.retract()));
        SmartDashboard.putData("Extend Elevator", new InstantCommand(() -> Subsystems.elevator.extend()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 
        // An ExampleCommand will run in autonomous
        // return new aprilAuto(right.getX(),right.getY());
        return autoManager.getSelectedCommand();
    }
    
}
