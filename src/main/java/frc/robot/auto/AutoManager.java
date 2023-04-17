package frc.robot.auto;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.auto.strategies.DebugAuto;
import frc.robot.auto.strategies.DoubleScore;
import frc.robot.auto.strategies.FlatOutRun;
import frc.robot.auto.strategies.OneAndAHalfBalance;
import frc.robot.auto.strategies.OneAndAHalfBalanceOtherSide;
import frc.robot.auto.strategies.OneAndAHalf;
import frc.robot.auto.strategies.OneAndAHalfOtherSide;
import frc.robot.auto.strategies.OverTheRainbow;
import frc.robot.auto.strategies.OverTheRainbowPlusPickup;
import frc.robot.auto.strategies.OverTheRainbowPlusVisionPickup;
import frc.robot.auto.strategies.ScoreAndBalance;
import frc.robot.auto.strategies.ScoreConeThenCube;
import frc.robot.auto.strategies.ScoreLowConeThenLowCone;
import frc.robot.auto.strategies.ScoredStraight;
import frc.robot.auto.strategies.TryForThree;
import frc.robot.auto.strategies.TwoScoreOtherSide;
import frc.robot.auto.strategies.VisionTestStrategy;
import frc.robot.subsystems.vision.Pipeline;

public class AutoManager {

    public enum AutoStrategies {
        DebugAuto, ExampleAuto, PDistTest, TestTrajectoryFactory, VisionTest,
        ScoreAndBalance, ScoreAndBalanceOtherSide, ScoredStraight, FlatOutRun, 
        OverTheRainbow, OverTheRainbowPlusPickup, OTRNext, OTRNextCube,
        ConeThenCube,
        DoubleScore, LowConeThenLowCone,
        TryForThree, 
        OneAndAHalfBalance, OneAndAHalfBalanceOtherSide, OneAndAHalfOtherSide, OneAndAHalf,
        TwoScoreOtherSide
    }

    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Supplier<Command>> strategyLookup = new HashMap<>();

    public AutoManager() {
        registerStrategy("Debug Auto", AutoStrategies.DebugAuto, DebugAuto::new);
        registerStrategy("Vision Test", AutoStrategies.VisionTest, VisionTestStrategy::new);
        // registerStrategy("exampleAuto", AutoStrategies.ExampleAuto, new exampleAuto(Subsystems.swerveSubsystem));
        // registerStrategy("pdist test", AutoStrategies.PDistTest, PDistTest::new);
        // registerStrategy("TestTrajectoryFactory", AutoStrategies.TestTrajectoryFactory, TestTrajectoryFactory::new, true);
        registerStrategy("Score And Balance", AutoStrategies.ScoreAndBalance, ScoreAndBalance::new, true);
        registerStrategy("Score And Balance Other Side", AutoStrategies.ScoreAndBalanceOtherSide, () -> new ScoreAndBalance(true));
        registerStrategy("Scored Straight", AutoStrategies.ScoredStraight, ScoredStraight::new);
        registerStrategy("FlatOutRun", AutoStrategies.FlatOutRun, FlatOutRun::new);
        // registerStrategy("Experimental DoubleScore", AutoStrategies.DoubleScore, DoubleScore::new);
        // registerStrategy("Score Cone then Cube", AutoStrategies.ConeThenCube, ScoreConeThenCube::new);
        registerStrategy("Over The Rainbow", AutoStrategies.OverTheRainbow, OverTheRainbow::new);
        registerStrategy("Over The Rainbow Plus Pickup", AutoStrategies.OverTheRainbowPlusPickup, OverTheRainbowPlusPickup::new);
        registerStrategy("OTRNext", AutoStrategies.OTRNext, OverTheRainbowPlusVisionPickup::new);
        registerStrategy("OTRNextCube", AutoStrategies.OTRNextCube, () -> new OverTheRainbowPlusVisionPickup(Pipeline.Cube));
        // registerStrategy("Double Low Cone Balance", AutoStrategies.LowConeThenLowCone, ScoreLowConeThenLowCone::new);
        // registerStrategy("Try For Three", AutoStrategies.TryForThree, TryForThree::new);
        registerStrategy("1.5 Balance", AutoStrategies.OneAndAHalfBalance, OneAndAHalfBalance::new);
        registerStrategy("1.5 Balance Other", AutoStrategies.OneAndAHalfBalanceOtherSide, OneAndAHalfBalanceOtherSide::new);
        registerStrategy("1.5 Other Stop", AutoStrategies.OneAndAHalfOtherSide, OneAndAHalfOtherSide::new);
        registerStrategy("1.5 Stop", AutoStrategies.OneAndAHalf, OneAndAHalf::new);
        registerStrategy("TwoScore Other", AutoStrategies.TwoScoreOtherSide, TwoScoreOtherSide::new);

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
