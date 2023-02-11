package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.DebugAuto;

public class AutoManager {
    public enum AutoStrategies {
        DebugAuto, DebugTimed, DebugPath, DebugRotate,
        TwoBallRight, TwoBallHangar, TwoBallCenter, 
        JustShoot,
        ShootFirstBall5Ball, FiveBall, FiveBallStrategyPartDeux,RIGHT5ball,LEFT5ball,
        ScrambleHangar, ScrambleHangarOneBall,
        OneBall, BlockOpponentOneBall,
        FourBall
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
        chooser.addOption("Debug Auto", AutoStrategies.DebugAuto);

        // Send selector Dashboard.  If it doesn't show in SD, you may need to change the name here.
        SmartDashboard.putData("Auto Selector", chooser);
        initializeAuto();
    }

    public void initializeAuto() {
        strategyLookup.put(AutoStrategies.DebugAuto, new DebugAuto());
    }

    public Command getSelectedCommand() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            switch (chooser.getSelected()) {
    
                case DebugAuto:
                    selected = new DebugAuto();
                    break;
    
                default:
                    selected = new InstantCommand();
            }
        }
        return selected;
    }

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown" );
    }

}
