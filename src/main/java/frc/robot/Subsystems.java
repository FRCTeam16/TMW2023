package frc.robot;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lifecycle;
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

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        swerveSubsystem = new Swerve();
        visionSubsystem = new VisionSubsystem();
        ledSubsystem = new LEDSubsystem();
        elevator = new Elevator();
        ramp = new Ramp();

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(elevator);
        lifecycleSubsystems.add(ramp);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
