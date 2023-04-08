package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PartIndicator extends SubsystemBase {

    public static final String KEY = "RequestedPart";


    public enum PartType {
        None(0),
        Cone(1),
        Cube(2);

        public int value;

        private PartType(int value) {
            this.value = value;
        }
    }

    
    // 00 - no request if red alliance
    // 01 - cone
    // 02 - cube
    // 11 - no request if blue allianc
    //  
    public PartType requestedPartType = PartType.None;
    
    public PartIndicator() {
    }

    public void requestPart(PartType partType) {
        this.requestedPartType = partType;
        SmartDashboard.putNumber(KEY, this.requestedPartType.value);
    }

    public boolean isConeRequested() { return PartType.Cone == this.requestedPartType; }
    public boolean isCubeRequested() { return PartType.Cube == this.requestedPartType; }
}
