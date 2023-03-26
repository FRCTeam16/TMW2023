package frc.robot.subsystems.DMS;

import org.junit.jupiter.api.Test;

public class TestMADDMS {
    
    @Test
    public void testBasicMAD() {
        DriveInfo<Double> data = new DriveInfo<Double>(1.23, 0.67, 1.73, 1.69);
        DMSStats dmsStats = new DMSStats();
        // workaround avg problem with one data point
        for (int i=0;i<1000;i++) {
            dmsStats.addDriveCurrent(data);
        }
        var result = dmsStats.calculateStatusMAD();
        assert result.FR == -1;
    }
}
