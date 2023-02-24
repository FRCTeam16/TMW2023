package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class ConfigureSoftLimits extends CommandBase {

    private final boolean enable;

    public ConfigureSoftLimits(boolean enable) {
        this.enable = enable;
    }

    @Override
    public void execute() {
        Subsystems.elevator.setSoftLimitsEnabled(this.enable);
        Subsystems.pivot.setSoftLimitsEnabled(this.enable);
        Subsystems.intake.setSoftLimitsEnabled(this.enable);
        System.out.println("ConfigureSoftLimits: " + enable);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
