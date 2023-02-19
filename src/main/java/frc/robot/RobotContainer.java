package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.aprilAuto;
import frc.robot.commands.ConfigureSoftLimits;
import frc.robot.commands.PoseElevator;
import frc.robot.commands.PosePivot;
import frc.robot.commands.PoseWrist;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Intake.WristPosition;
import frc.robot.subsystems.Pivot.PivotPosition;

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

    private final JoystickButton intake    = new JoystickButton(right,    1);
    // private final JoystickButton padIntake = new JoystickButton(gamepad, 6);
    private final JoystickButton eject     = new JoystickButton(left,   1);
    // private final JoystickButton padEject  = new JoystickButton(gamepad, 5);
      
    private final Trigger wristOpenLoopUp   = new JoystickButton(right, 4);
    private final Trigger wristOpenLoopDown = new JoystickButton(right, 3);
    private final Trigger wristTempDown     = new JoystickButton(right, 2);

    private final JoystickButton openHand  = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);
    private final JoystickButton closeHand = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);

    private final Trigger rotateArmUp   = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kLeftY.value) >  0.10);
    private final Trigger rotateArmDown = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kLeftY.value) < -0.10);
    
    private final Trigger elevatorForward = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) < -0.10);
    private final Trigger elevatorReverse = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) >  0.10);

    private final JoystickButton extendRamp  = new JoystickButton(gamepad, XboxController.Button.kX.value);
    private final JoystickButton retractRamp = new JoystickButton(gamepad, XboxController.Button.kY.value);

    private final JoystickButton zeroGyro     = new JoystickButton(gamepad, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(right, 3);


    /* Subsystems */
    private final Subsystems subsystems = Subsystems.getInstance();

    private final PneumaticHub pneumaticHub = new PneumaticHub();

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
        // Configure pneumatic pressures
        pneumaticHub.enableCompressorAnalog(60, 100);
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
        
        // padIntake.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand())).onFalse(new InstantCommand(() -> Subsystems.intake.OpenHand()));

        openHand.onTrue(new InstantCommand(() -> Subsystems.intake.OpenHand()));
        closeHand.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand()));
    }


    private void configureDashboardButtons() {
        SmartDashboard.putData("Zero Gyro", new RunWithDisabledInstantCommand(Subsystems.swerveSubsystem::zeroGyro));

        SmartDashboard.putData("Zero Elevator Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.elevator.zeroElevatorEncoder()));
        SmartDashboard.putData("Zero Wrist Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.intake.zeroWristEncoder()));
        SmartDashboard.putData("Zero Pivot Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.pivot.zeroPivotEncoder()));
        // SmartDashboard.putData("Retract Elevator", new InstantCommand(() -> Subsystems.elevator.retract()));
        // SmartDashboard.putData("Extend Elevator", new InstantCommand(() -> Subsystems.elevator.extend()));

        SmartDashboard.putData("Elev Move to Zero", new PoseElevator(ElevatorPosition.Down));
        SmartDashboard.putData("Elev Move to GroundPickup", new PoseElevator(ElevatorPosition.GroundPickup));

        SmartDashboard.putData("Pivot Move to Zero", new PosePivot(PivotPosition.Vertical));
        SmartDashboard.putData("Pivot Move to Horizontal", new PosePivot(PivotPosition.Horizontal));
        SmartDashboard.putData("Pivot Move to GroundPickup", new PosePivot(PivotPosition.GroundPickup));

        SmartDashboard.putData("Wrist Move to Zero", new PoseWrist(WristPosition.Vertical));
        SmartDashboard.putData("Wrist Move to GroundPickup", new PoseWrist(WristPosition.GroundPickup));

        SmartDashboard.putData("Enable Soft Limits", new ConfigureSoftLimits(true));
        SmartDashboard.putData("Disable Soft Limits", new ConfigureSoftLimits(false));
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

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
    }

}
