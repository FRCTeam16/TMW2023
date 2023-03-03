package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;
import frc.robot.SwerveModule;

public class XWHeelLock extends CommandBase {

    public XWHeelLock() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void execute() {
        SwerveModule[] mods = Subsystems.swerveSubsystem.mSwerveMods;
        mods[0].setWheelAngleManually(45);
        mods[1].setWheelAngleManually(-45);
        mods[2].setWheelAngleManually(-45);
        mods[3].setWheelAngleManually(45);
    }

}
