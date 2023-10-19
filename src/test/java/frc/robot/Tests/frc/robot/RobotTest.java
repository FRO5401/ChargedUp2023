package frc.robot.Tests.frc.robot;
import frc.robot.Subsystems.DriveBase;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class RobotTest {

    @Test
    public void mathBrokeTest() {
       assertEquals(1, 2);

    }

    @Test
    public void mathWorkTest() {
        assertEquals(1, 1);
    }

    @Test
    public void robotTest(){
        DriveBase driveBase = new DriveBase();
        assertEquals(driveBase.getAdjustedSpeed(0, 0), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(0.5, 0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(-0.5, -0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(0.5, -0.5), 0.01);
        assertEquals(driveBase.getAdjustedSpeed(-0.5, 0.5), 0.01);
    }
}
