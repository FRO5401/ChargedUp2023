package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class Lidar {
	/*
	 * Adjust the Calibration Offset to compensate for differences in each unit.
	 * We've found this is a reasonably constant value for readings in the 25 cm to
	 * 600 cm range.
	 * You can also use the offset to zero out the distance between the sensor and
	 * edge of the robot.
	 */
	private static final int CALIBRATION_OFFSET = 0;

	private Counter counter;

	public Lidar(int i) {
		counter = new Counter(i);
		counter.setMaxPeriod(1.0);
		counter.setSemiPeriodMode(true);
		counter.reset();
	}

	public double getDistance() {
		double cm;
		cm = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
		return cm;
	}

	public boolean getDistanceInRange() {
		double distance = getDistance();
		if (distance <= 35 && distance > 32) {
			return true;
		} else {
			return false;
		}
	}

}