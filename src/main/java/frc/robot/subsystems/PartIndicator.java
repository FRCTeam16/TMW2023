package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private DigitalOutput requestPin1 = new DigitalOutput(1);
    private DigitalOutput requestPin2 = new DigitalOutput(2);

    private PartType requestedPartType = PartType.None;
    
    public PartIndicator() {
    }

    public void requestPart(PartType partType) {
        this.requestedPartType = partType;
        SmartDashboard.putNumber(KEY, this.requestedPartType.value);
    }

    @Override
    public void periodic() {
        switch (requestedPartType) {
            case None:
                boolean colorSignal  = DriverStation.getAlliance() == Alliance.Blue;
                requestPin1.set(colorSignal);
                requestPin2.set(colorSignal);
                break;
            case Cone:
                requestPin1.set(false);
                requestPin2.set(true);
                break;
            case Cube:
                requestPin1.set(true);
                requestPin2.set(false);
                break;
        }
    }
}
