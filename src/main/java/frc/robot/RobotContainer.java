package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.autos.aprilAuto;
import frc.robot.commands.ConfigureSoftLimits;
import frc.robot.commands.RequestPart;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.RequestPart.PartType;
import frc.robot.commands.pose.PoseElevator;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.commands.pose.PosePivot;
import frc.robot.commands.pose.PoseWrist;
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


    //
    // Intake
    //
    private final JoystickButton april     = new JoystickButton(left,    2);

    private final JoystickButton intake    = new JoystickButton(right,    1);
    private final JoystickButton eject     = new JoystickButton(left,   1);
      
    private final Trigger wristOpenLoopUp   = new JoystickButton(right, 4);
    private final Trigger wristOpenLoopDown = new JoystickButton(right, 3);
    private final Trigger wristTempDown     = new JoystickButton(right, 2);

    private final JoystickButton openHandJoy = new JoystickButton(left, 3);
    private final JoystickButton closeHandJoy = new JoystickButton(left, 4);

    //
    // Pivot
    //
    private final Trigger rotateArmUp   = new Trigger(() -> gamepad.getLeftY() >  0.10);
    private final Trigger rotateArmDown = new Trigger(() -> gamepad.getLeftY() < -0.10);
    
    //
    // Elevator
    //
    private final Trigger elevatorForward = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) < -0.10);
    private final Trigger elevatorReverse = new Trigger(() -> gamepad.getRawAxis(XboxController.Axis.kRightY.value) >  0.10);
    private final JoystickButton elevatorRest = new JoystickButton(gamepad, XboxController.Button.kBack.value);


    //
    // Poses
    //
    private final Trigger singleSubstationPose = new Trigger(() -> gamepad.getPOV() == 180);
    private final Trigger doubleSubstationPose = new Trigger(() -> gamepad.getPOV() == 0);
    private final JoystickButton scoreConeHighPose = new JoystickButton(gamepad, XboxController.Button.kY.value);
    private final JoystickButton scoreConeMidPose = new JoystickButton(gamepad, XboxController.Button.kX.value);
    private final JoystickButton groundPickupPose = new JoystickButton(gamepad, XboxController.Button.kB.value);
    private final JoystickButton stowPose = new JoystickButton(gamepad, XboxController.Button.kA.value);


    //
    // General Commands
    //
    private final JoystickButton zeroGyro     = new JoystickButton(gamepad, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(left, 8);
    private final JoystickButton enableLimelight = new JoystickButton(left, 16);
    private final JoystickButton disableLimelight = new JoystickButton(left, 15);


    /* Subsystems */
    private final Subsystems subsystems = Subsystems.getInstance();
    private final AutoManager autoManager = new AutoManager();

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
        zeroGyro.onTrue(new InstantCommand(() -> Subsystems.swerveSubsystem.zeroGyro()).ignoringDisable(true));

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
        elevatorRest.onTrue(new InstantCommand(Subsystems.elevator::restOpenLoop));


        singleSubstationPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.SingleSubstation))));
        doubleSubstationPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.DoubleSubstation))));
        scoreConeHighPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.ScoreHighCone))));
        scoreConeMidPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.ScoreMidCone))));
        groundPickupPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.GroundPickup))));
        stowPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.Stow))));
        
        
        
        /* Intake Controls */
        intake.onTrue(new InstantCommand(()    -> Subsystems.intake.intake())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));
        eject.onTrue(new InstantCommand(()     -> Subsystems.intake.eject())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));

        wristOpenLoopUp.onTrue(new InstantCommand(()   -> Subsystems.intake.raiseWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
        wristOpenLoopDown.onTrue(new InstantCommand(() -> Subsystems.intake.lowerWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
        wristTempDown.onTrue(new InstantCommand(()     -> Subsystems.intake.storeAndScore())).onFalse(new InstantCommand(() -> Subsystems.intake.restoreStoredSetpoint()));
        
        // padIntake.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand())).onFalse(new InstantCommand(() -> Subsystems.intake.OpenHand()));

        openHandJoy.onTrue(new InstantCommand(() -> Subsystems.intake.OpenHand()));
        closeHandJoy.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand()));

        enableLimelight.onTrue(new InstantCommand(() -> Subsystems.visionSubsystem.enable()).ignoringDisable(true));
        disableLimelight.onTrue(new InstantCommand(() -> Subsystems.visionSubsystem.disable()).ignoringDisable(true));
    }


    private void configureDashboardButtons() {
        SmartDashboard.putData("Zero Gyro", new RunWithDisabledInstantCommand(Subsystems.swerveSubsystem::zeroGyro));

        SmartDashboard.putData("Zero Elevator Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.elevator.zeroElevatorEncoder()));
        SmartDashboard.putData("Zero Wrist Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.intake.zeroWristEncoder()));
        SmartDashboard.putData("Zero Pivot Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.pivot.zeroPivotEncoder()));

        SmartDashboard.putData("Elev Move to Zero", new PoseElevator(ElevatorPosition.Down));
        SmartDashboard.putData("Elev Move to GroundPickup", new PoseElevator(ElevatorPosition.GroundPickup));

        SmartDashboard.putData("Pivot Move to Zero", new PosePivot(PivotPosition.Vertical));
        SmartDashboard.putData("Pivot Move to Horizontal", new PosePivot(PivotPosition.Horizontal));
        SmartDashboard.putData("Pivot Move to GroundPickup", new PosePivot(PivotPosition.GroundPickup));

        SmartDashboard.putData("Wrist Move to Zero", new PoseWrist(WristPosition.Vertical));
        SmartDashboard.putData("Wrist Move to GroundPickup", new PoseWrist(WristPosition.GroundPickup));

        SmartDashboard.putData("Enable Soft Limits", new ConfigureSoftLimits(true));
        SmartDashboard.putData("Disable Soft Limits", new ConfigureSoftLimits(false));

        SmartDashboard.putNumber(RequestPart.KEY, RequestPart.PartType.None.value);
        SmartDashboard.putData("Request No Part", new RequestPart(PartType.None).ignoringDisable(true));
        SmartDashboard.putData("Request Cube",  new RequestPart(PartType.Cube).ignoringDisable(true));
        SmartDashboard.putData("Request Cone", new RequestPart(PartType.Cone).ignoringDisable(true));

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

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
    }

}
