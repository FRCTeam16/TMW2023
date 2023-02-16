package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ramp extends SubsystemBase implements Lifecycle {
    
    private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);  // FIXME: Is this true
    private boolean extended = false;

    public void forward() {
        extended = true;
    }

    public void back() {
        extended = false;
    }

    @Override
    public void periodic() {
        solenoid.set(extended);
    }
}
