package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final Joystick left    = new Joystick(0);
    private final Joystick right   = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);


    private final JoystickButton april     = new JoystickButton(left,    2);
    private final JoystickButton intake    = new JoystickButton(left,    1);
    // private final JoystickButton padIntake = new JoystickButton(gamepad, 5);
    private final JoystickButton eject     = new JoystickButton(right,   1);
    // private final JoystickButton padEject  = new JoystickButton(gamepad, 4);
      
    private final Trigger wristOpenLoopUp   = new JoystickButton(right, 4);
    private final Trigger wristOpenLoopDown = new JoystickButton(right, 3);
    private final Trigger wristTempDown     = new JoystickButton(right, 5); // FIXME: what button?

    // private final Trigger PadIntake     = new Trigger(() -> gamepad.getRawButton(1));
    private final Trigger rotateArmUp   = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kLeftY.value) >  0.10);
    private final Trigger rotateArmDown = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kLeftY.value) < -0.10);
    
    private final Trigger elevatorForward = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) < -0.10);
    private final Trigger elevatorReverse = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) >  0.10);

    private final JoystickButton extendRamp  = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);
    private final JoystickButton retractRamp = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);

    private final JoystickButton zeroGyro     = new JoystickButton(gamepad, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(right, 2);


    /* Subsystems */
    private final Subsystems subsystems = Subsystems.getInstance();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        Subsystems.swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                Subsystems.swerveSubsystem, 
                () -> -right.getY(),
                () -> -right.getX(), 
                () ->  left.getX(), 
                () ->  robotCentric.getAsBoolean()
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


        rotateArmUp.onTrue(new InstantCommand(() -> Subsystems.pivot.openLoopUp()))
                  .onFalse(new InstantCommand(() -> Subsystems.pivot.holdPosition()));

        rotateArmDown.onTrue(new InstantCommand(() -> Subsystems.pivot.openLoopDown()))
                    .onFalse(new InstantCommand(() -> Subsystems.pivot.holdPosition()));
            

        elevatorForward
            .onTrue(new InstantCommand(()  -> Subsystems.elevator.forward()))
            .onFalse(new InstantCommand(() -> Subsystems.elevator.stop()));
        elevatorReverse
            .onTrue(new InstantCommand(()  -> Subsystems.elevator.reverse()))
            .onFalse(new InstantCommand(() -> Subsystems.elevator.stop()));


        extendRamp.onTrue(new InstantCommand(()  -> Subsystems.ramp.forward()));
        retractRamp.onTrue(new InstantCommand(() -> Subsystems.ramp.back()));

        
        /* Intake Controls */
        intake.onTrue(new InstantCommand(()    -> Subsystems.intake.intake())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));
        // padIntake.onTrue(new InstantCommand(() -> Subsystems.intake.intake())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));
        eject.onTrue(new InstantCommand(()     -> Subsystems.intake.eject())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));
        // padEject.onTrue(new InstantCommand(()  -> Subsystems.intake.eject())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));

        wristOpenLoopUp.onTrue(new InstantCommand(()   -> Subsystems.intake.raiseWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
        wristOpenLoopDown.onTrue(new InstantCommand(() -> Subsystems.intake.lowerWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
        wristTempDown.onTrue(new InstantCommand(()     -> Subsystems.intake.lowerWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.raiseWristOpenLoop()));
        
        // PadIntake.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand())).onFalse(new InstantCommand(() -> Subsystems.intake.OpenHand()));
    }


    private void configureDashboardButtons() {
        SmartDashboard.putData("Zero Elevator Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.elevator.zeroElevatorEncoder()));
        SmartDashboard.putData("Zero Pivot Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.pivot.zeroPivotEncoder()));
        SmartDashboard.putData("Zero Wrist Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.intake.zeroWristEncoder()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 
        // An ExampleCommand will run in autonomous
        return new aprilAuto(right.getX(),right.getY());
    }

}
