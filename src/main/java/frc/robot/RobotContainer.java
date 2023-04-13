package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.autos.aprilAuto;
import frc.robot.commands.Balance;
import frc.robot.commands.ConfigureSoftLimits;
import frc.robot.commands.EnableDriverCamera;
import frc.robot.commands.EnableImageProcessing;
import frc.robot.commands.RunDMSCommand;
import frc.robot.commands.RunWithDisabledInstantCommand;
import frc.robot.commands.SchedulePose;
import frc.robot.commands.ScoreHighCone;
import frc.robot.commands.SimpleTimedDriveBack;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.XWHeelLock;
import frc.robot.commands.ZeroAndSetOffsetCommand;
import frc.robot.commands.auto.RotateToAngle;
import frc.robot.commands.pose.PoseManager.Pose;
import frc.robot.subsystems.Pivot.PivotPosition;
import frc.robot.subsystems.util.VisionAlignmentHelper;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.TargetInfo;

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
    // Drive state management
    //
    private boolean lockAngleEnabled = false;
    private double lockAngle = 0;
    private boolean visionAlignmentEnabled = false;
    private VisionAlignmentHelper visionAlignmentHelper = new VisionAlignmentHelper();


    //
    // Intake
    //
    private final JoystickButton april     = new JoystickButton(left,    13);

    private final JoystickButton intake    = new JoystickButton(right,    1);
    private final JoystickButton eject     = new JoystickButton(left,   1);
    private final JoystickButton lockAngle180 = new JoystickButton(left, 2);
    private final Trigger lockAngleN90 = new Trigger(() -> left.getPOV() >= 0);
    private final Trigger lockAngleN0 = new Trigger(() -> right.getPOV() >= 0);
    private final Trigger visionAlign = new JoystickButton(right, 11);
    private final Trigger visionAlignCmd = new JoystickButton(right, 12);
    private final Trigger visionAlignCmdZero = new JoystickButton(right, 13);
      
    private final Trigger wristOpenLoopDown = new JoystickButton(gamepad, XboxController.Button.kLeftBumper.value);
    private final Trigger wristOpenLoopUp   = new JoystickButton(gamepad, XboxController.Button.kRightBumper.value);
    private final Trigger wristTempDown     = new JoystickButton(right, 2);
    private final Trigger wristElevatorBumpSelector = new Trigger(this::wristOrElevatorOffset);

    private final JoystickButton openHandJoy = new JoystickButton(left, 4);
    private final JoystickButton closeHandJoy = new JoystickButton(left, 3);

    // Part Request
    private final JoystickButton requestCone = new JoystickButton(right, 3);
    private final JoystickButton requestCube = new JoystickButton(right, 4);

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
    
    private final JoystickButton xLock = new JoystickButton(right, 10);

    //
    // General Commands
    //
    private final JoystickButton zeroGyro     = new JoystickButton(gamepad, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(left, 8);
    //private final JoystickButton enableLimelight = new JoystickButton(left, 16);
    //private final JoystickButton disableLimelight = new JoystickButton(left, 15);
    private final JoystickButton StickPoseSS = new JoystickButton(left, 16);
    private final JoystickButton StickPoseDS = new JoystickButton(left, 15);
    private final JoystickButton detectScoreHighConePositionEnable = new JoystickButton(left, 14);
    private final Trigger detectScorePositionTrigger = new Trigger(() -> Subsystems.visionSubsystem.getScorePositionDetector().inRequestedScoringPosition());
    private final JoystickButton runDMS = new JoystickButton(right, 8);

    //
    private DigitalInput dmsButtonInput = new DigitalInput(1);
    private final Trigger dmsButton = new Trigger(dmsButtonInput::get);

    /* Subsystems */
    private final Subsystems subsystems = Subsystems.getInstance();
    private final AutoManager autoManager = new AutoManager();

    private final PneumaticHub pneumaticHub = new PneumaticHub();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        Subsystems.swerveSubsystem.gyro.setGyroOffset(180);

        Subsystems.swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                Subsystems.swerveSubsystem,
                () -> -right.getY(),
                () -> getStrafeValue(),
                () ->  -left.getX(),
                () ->  robotCentric.getAsBoolean(),
                () -> this.getLockAngleEnabled(),
                () -> this.getLockAngle()));

        // Configure the button bindings
        configureButtonBindings();
        // Configure software buttons
        configureDashboardButtons();
        // Configure pneumatic pressures
        pneumaticHub.enableCompressorAnalog(80, 100);
    }

    public double getLockAngle() { return this.lockAngle; }
    
    public boolean getLockAngleEnabled() { return this.lockAngleEnabled; }
    private void enableLockAngle(double target) { this.lockAngle = target; this.lockAngleEnabled = true; }
    private void disableLockAngle() { this.lockAngleEnabled = false; }

    /**
     * Strafe supplier for teleop.  Can be joystick or vision
     */
    public double getStrafeValue() {
        if (visionAlignmentEnabled) {
            double direction = (Math.abs(Subsystems.swerveSubsystem.getYaw().getDegrees()) < 90) ? 1 : -1;
            return direction * visionAlignmentHelper.calculate();
         } else {
            return -right.getX();
         } 
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

        lockAngle180
            .onTrue(new InstantCommand(() -> this.enableLockAngle(180)))
            .onFalse(new InstantCommand(() -> this.disableLockAngle()));

        lockAngleN90
            .onTrue(new InstantCommand(() -> this.enableLockAngle( DriverStation.getAlliance() == Alliance.Red ? -90 : 90)))
            .onFalse(new InstantCommand(() -> this.disableLockAngle()));

        lockAngleN0
            .onTrue(new InstantCommand(() -> this.enableLockAngle(0)))
            .onFalse(new InstantCommand(() -> this.disableLockAngle()));

        visionAlign
            .onTrue(new EnableImageProcessing(Pipeline.Cone)
            .andThen(new InstantCommand(() -> this.visionAlignmentEnabled = true)))
            .onFalse(new EnableDriverCamera()
            .andThen(new InstantCommand(() -> this.visionAlignmentEnabled = false)));

        visionAlignCmd.whileTrue(new VisionAlign());
        visionAlignCmdZero.whileTrue(new VisionAlign().withRobotAngle(0));


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
        StickPoseSS.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.SingleSubstation))));

        doubleSubstationPose.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.DoubleSubstation))));
        StickPoseDS.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.DoubleSubstation))));
        
        scoreConeHighPose.onTrue(new InstantCommand(()    -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.ScoreHighCone))));
        scoreConeMidPose.onTrue(new InstantCommand(()     -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.ScoreMidCone))));
        groundPickupPose.onTrue(new InstantCommand(()     -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.GroundPickup))));
        stowPose.onTrue(new InstantCommand(()             -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.Stow))));

        // DEBUG
        new JoystickButton(left, 5)
            .onTrue(new SimpleTimedDriveBack(2.0)
            .andThen(new ScoreHighCone())
            .andThen(new SchedulePose(Pose.SingleSubstation))
        );
        
        xLock.whileTrue(new XWHeelLock());

        runDMS.whileTrue(new RunDMSCommand());
        dmsButton.whileTrue(new RunDMSCommand());
        
        /* Intake Controls */
        intake.onTrue(new InstantCommand(()    -> Subsystems.intake.intake())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));
        eject.onTrue(new InstantCommand(()     -> Subsystems.intake.eject())).onFalse(new InstantCommand(() -> Subsystems.intake.stopIntake()));

        wristOpenLoopUp.onTrue(new InstantCommand(()   -> Subsystems.intake.raiseWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
        wristOpenLoopDown.onTrue(new InstantCommand(() -> Subsystems.intake.lowerWristOpenLoop())).onFalse(new InstantCommand(() -> Subsystems.intake.holdWrist()));
       // wristTempDown.onTrue(new InstantCommand(()     -> Subsystems.intake.storeAndScore())).onFalse(new InstantCommand(() -> Subsystems.intake.restoreStoredSetpoint()));

        wristTempDown.and(wristElevatorBumpSelector)
            .whileTrue(new InstantCommand(() -> Subsystems.intake.storeAndScore()))
            .onFalse(new InstantCommand(() -> Subsystems.intake.restoreStoredSetpoint()));

        wristTempDown.and(wristElevatorBumpSelector.negate())
            .whileTrue(new InstantCommand(() -> Subsystems.pivot.addOffset()))
            .onFalse(new InstantCommand(() -> Subsystems.pivot.setPivotPosition(PivotPosition.GroundPickup)));
        

        requestCone.onTrue(new InstantCommand(() -> Subsystems.partIndicator.requestPart(frc.robot.subsystems.PartIndicator.PartType.Cone)).ignoringDisable(true));
        requestCube.onTrue(new InstantCommand(() -> Subsystems.partIndicator.requestPart(frc.robot.subsystems.PartIndicator.PartType.Cube)).ignoringDisable(true));
        requestCone.and(requestCube).onTrue(new InstantCommand(() -> Subsystems.partIndicator.requestPart(frc.robot.subsystems.PartIndicator.PartType.None)).ignoringDisable(true));

        openHandJoy.onTrue(new InstantCommand(() -> Subsystems.intake.OpenHand()));
        closeHandJoy.onTrue(new InstantCommand(() -> Subsystems.intake.CloseHand()));

        //
        // Vision Subsytems - TEMP DISABLED FOR OTHER CONTROLLS
        //
        //enableLimelight.onTrue(new InstantCommand(() -> Subsystems.visionSubsystem.enable()).ignoringDisable(true));
        //disableLimelight.onTrue(new InstantCommand(() -> Subsystems.visionSubsystem.disable()).ignoringDisable(true));

        // Vision-based automatic scoring
        detectScoreHighConePositionEnable
            .onTrue(new InstantCommand(() -> Subsystems.visionSubsystem.getScorePositionDetector().setCurrentTarget(TargetInfo.HighPole)))
            .onFalse(new InstantCommand(() -> Subsystems.visionSubsystem.getScorePositionDetector().setCurrentTarget(null)));
        detectScoreHighConePositionEnable.and(detectScorePositionTrigger).onTrue(new InstantCommand(() -> new XWHeelLock()));

        new JoystickButton(right, 16).whileTrue(new Balance());
    }


    /**
     * True for wrist bump, false for elevator bump
     * @return
     */
    private final boolean wristOrElevatorOffset() {

        if (Subsystems.poseManager.getCurrentPose() == Pose.GroundPickup && !Subsystems.intake.isHandOpen()) {
            return false;
        } else {
            return true;
        }
    }

    private void configureDashboardButtons() {
        SmartDashboard.putData("Zero Gyro", new RunWithDisabledInstantCommand(Subsystems.swerveSubsystem::zeroGyro));

        SmartDashboard.putData("Zero Elevator Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.elevator.zeroElevatorEncoder()));
        SmartDashboard.putData("Zero Wrist Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.intake.zeroWristEncoder()));
        SmartDashboard.putData("Zero Pivot Encoder", new RunWithDisabledInstantCommand(() -> Subsystems.pivot.zeroPivotEncoder()));

        SmartDashboard.putData("Enable Soft Limits", new ConfigureSoftLimits(true));
        SmartDashboard.putData("Disable Soft Limits", new ConfigureSoftLimits(false));

        SmartDashboard.putData("Test Rotation", new RotateToAngle(45));
        SmartDashboard.putData("Move To Zero", new InstantCommand(() -> CommandScheduler.getInstance().schedule(Subsystems.poseManager.getPose(Pose.Zero))));


        SmartDashboard.putData("Zero With 180 Offset", new ZeroAndSetOffsetCommand(180));

        SmartDashboard.putData("Vision.Enable Limelight", new RunWithDisabledInstantCommand(Subsystems.visionSubsystem::enable));
        SmartDashboard.putData("Vision.Disable Limelight", new RunWithDisabledInstantCommand(Subsystems.visionSubsystem::disable));
        SmartDashboard.putData("Vision.Select April Pipe", Subsystems.visionSubsystem.selectPipeline(Pipeline.April));
        SmartDashboard.putData("Vision.Select Retro Pipe High", Subsystems.visionSubsystem.selectPipeline(Pipeline.RetroHigh));
        SmartDashboard.putData("Vision.Select Retro Pipe Low", Subsystems.visionSubsystem.selectPipeline(Pipeline.Cone));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 
        return autoManager.getSelectedCommand();
    }

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
    }


    public void periodic() {
        SmartDashboard.putString("CurrentPose", Subsystems.poseManager.getCurrentPose().toString());
        SmartDashboard.putNumber("CurrentYaw", Subsystems.swerveSubsystem.getYaw().getDegrees());
        SmartDashboard.putNumber("Pitch", Subsystems.swerveSubsystem.gyro.getPitch());
        autoManager.showSelectedAuto();
    }

}
