package frc.robot.auto;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.FlatOutRun;
import frc.robot.auto.strategies.ScoreAndBalance;
import frc.robot.auto.strategies.ScoredStraight;
import frc.robot.auto.strategies.TestTrajectoryFactory;
import frc.robot.autos.PDistTest;
import frc.robot.auto.strategies.DoubleScore;

public class AutoManager {

    public enum AutoStrategies {
        DebugAuto, ExampleAuto, PDistTest, TestTrajectoryFactory, 
        ScoreAndBalance, ScoreAndBalanceOtherSide, ScoredStraight, FlatOutRun, DoubleScore
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Supplier<Command>> strategyLookup = new HashMap<>();

    public AutoManager() {
        registerStrategy("Debug Auto", AutoStrategies.DebugAuto, DebugAuto::new);
        // registerStrategy("exampleAuto", AutoStrategies.ExampleAuto, new exampleAuto(Subsystems.swerveSubsystem));
        // registerStrategy("pdist test", AutoStrategies.PDistTest, PDistTest::new);
        // registerStrategy("TestTrajectoryFactory", AutoStrategies.TestTrajectoryFactory, TestTrajectoryFactory::new, true);
        registerStrategy("Score And Balance", AutoStrategies.ScoreAndBalance, ScoreAndBalance::new, true);
        registerStrategy("Score And Balance Other Side", AutoStrategies.ScoreAndBalanceOtherSide, () -> new ScoreAndBalance(true));
        registerStrategy("Scored Straight", AutoStrategies.ScoredStraight, ScoredStraight::new);
        registerStrategy("FlatOutRun", AutoStrategies.FlatOutRun, FlatOutRun::new);
        registerStrategy("DoubleScore", AutoStrategies.DoubleScore, DoubleScore::new);

        // Send selector Dashboard.  If it doesn't show in SD, you may need to change the name here.
        SmartDashboard.putData("Auto Selector", chooser);
    }

    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy) {
        registerStrategy(displayName, strategyEnum, strategy, false);
    }

    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy, boolean isDefault) {
        if (isDefault) {
            chooser.setDefaultOption(displayName, strategyEnum);
        } else {
            chooser.addOption(displayName, strategyEnum);
        }
        strategyLookup.put(strategyEnum, strategy);
    }

    /**
     * Returns the currently selected autonomous strategy
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedCommand() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected()).get();
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
