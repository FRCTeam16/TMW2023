package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;

public class ConfigureSoftLimits extends CommandBase {

    private final boolean enable;

    public ConfigureSoftLimits(boolean enable) {
        this.enable = enable;
        // this.ignoringDisable(true);
    }

    @Override
    public void execute() {
        Subsystems.elevator.setSoftLimitsEnabled(this.enable);
        Subsystems.pivot.setSoftLimitsEnabled(this.enable);
        Subsystems.intake.setSoftLimitsEnabled(this.enable);
        System.out.println("ConfigureSoftLimits: " + enable);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
