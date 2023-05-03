package frc.robot;
import frc.robot.Subsystems.DriveBase;

public class RobotTestTest {

        
    @Test
    public void test() {
        DriveBase driveBase = new DriveBase();
        assertEquals(driveBase.getAdjustedSpeed(0, 0), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(0.5, 0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(-0.5, -0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(0.5, -0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(-0.5, 0.5), 0.01);

    }
}
    