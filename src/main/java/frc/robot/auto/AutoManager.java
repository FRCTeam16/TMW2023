package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.DebugAuto;
import frc.robot.autos.exampleAuto;

public class AutoManager {

    public enum AutoStrategies {
        DebugAuto, ExampleAuto
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
        registerStrategy("Debug Auto", AutoStrategies.DebugAuto, new DebugAuto());
        registerStrategy("exampleAuto", AutoStrategies.ExampleAuto, new exampleAuto(Subsystems.swerveSubsystem));

        // Send selector Dashboard.  If it doesn't show in SD, you may need to change the name here.
        SmartDashboard.putData("Auto Selector", chooser);
    }

    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Command strategy) {
        chooser.addOption(displayName, strategyEnum);
        strategyLookup.put(strategyEnum, strategy);
    }

    /**
     * Returns the currently selected autonomous strategy
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedCommand() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            // Missing autonomous key in lookup map
            selected = new InstantCommand(() -> System.out.println("ERROR: Could not find requested auto: " + chooser.getSelected()));
        }
        return selected;
    }

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown" );
    }

}
