package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.commands.pose.PoseManager;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Global subsystem declaration and tracking to allow easier injection
 * for commands
 */
public class Subsystems {
    private static Subsystems instance;
    public static Swerve swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Elevator elevator;
    public static Ramp ramp;
    public static Pivot pivot;
    public static Intake intake;
    
    // not real subsystems
    public static PoseManager poseManager;
    
    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        swerveSubsystem = new Swerve();
        visionSubsystem = new VisionSubsystem();
        ledSubsystem = new LEDSubsystem();
        elevator = new Elevator();
        ramp = new Ramp();
        pivot = new Pivot();
        intake = new Intake();
        poseManager = new PoseManager();


        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(elevator);
        lifecycleSubsystems.add(ramp);
        lifecycleSubsystems.add(pivot);
        lifecycleSubsystems.add(intake);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
