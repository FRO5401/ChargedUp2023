package frc.robot.Subsystems;

import java.util.TimerTask;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lidar extends SubsystemBase {
	private I2C i2c;
	private byte[] distanceArray;
	private boolean started;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public Lidar(Port i) {
		i2c = new I2C(i, LIDAR_ADDR);
		
		distanceArray = new byte[2];
		
    }
	
	// Distance in cm
	public int convertDistance() {
		return (int)Integer.toUnsignedLong(distanceArray[0] << 8) + Byte.toUnsignedInt(distanceArray[1]);
	}

	public double getDistance() {
		return (((double)convertDistance())/100);
	}
    
    public void start()
    {
        started = true;
    }
	
	public void stop() {
		started = false;
	}
	
	// Update distance variable
	public void update() {
        if (started == true) {
			i2c.write(0x04, 0x08 | 32);
			i2c.write(0x11, 0xff);
            i2c.write(LIDAR_CONFIG_REGISTER, 0x04); 
            Timer.delay(0.04);
            i2c.read(LIDAR_DISTANCE_REGISTER, 2, distanceArray); 
			Timer.delay(0.02); 
        }    
    }
    public void reportLidarDistance()
    {
        System.out.println(getDistance());
    }
	
}    