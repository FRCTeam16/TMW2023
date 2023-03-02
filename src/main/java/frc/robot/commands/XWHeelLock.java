package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems;

public class XWHeelLock extends CommandBase {

    public XWHeelLock() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void execute() {
        SwerveModuleState[] state = new SwerveModuleState[4];
       
        state[0] = new SwerveModuleState();
        state[0].angle = Rotation2d.fromDegrees(45);
        state[1] = new SwerveModuleState();
        state[0].angle = Rotation2d.fromDegrees(-45);
        state[2] = new SwerveModuleState();
        state[0].angle = Rotation2d.fromDegrees(-45);
        state[3] = new SwerveModuleState();
        state[0].angle = Rotation2d.fromDegrees(45);

        Subsystems.swerveSubsystem.setModuleStates(state);
    }

}
