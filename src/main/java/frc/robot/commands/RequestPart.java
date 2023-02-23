package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class RequestPart extends CommandBase {
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

    private final PartType partType;

    public RequestPart(PartType partType) {
        this.partType = partType;
    }

    @Override
    public void execute() {
        System.out.println("Requesting part: " + this.partType);
        SmartDashboard.putNumber(KEY, this.partType.value);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
