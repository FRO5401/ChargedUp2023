package frc.robot.Commands.actions;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Subsystems.DriveBase;

public class AutoDriveVision extends CommandBase {

    private DriveBase drivebase;
	private double angle, desiredDistance, autoDriveSpeed, distanceTraveled; //Can declare variables next to each other
	private boolean doneTraveling;

	public AutoDriveVision(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);
        doneTraveling = false;
        drivebase = passedDrivebase;
		desiredDistance = DistanceInput;
		autoDriveSpeed = SpeedInput;
        distanceTraveled = 0;
        addRequirements(drivebase);
	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {

        drivebase.resetEncoders();
        drivebase.resetGyroAngle();

        distanceTraveled = 0;

    } 

	// Called repeatedly when this Command is scheduled to run
    /*
    @Override
    public void execute() {
        var result = drivebase.getCamera().getLatestResult();

            if (result.hasTargets()) {
                // First calculate range
                double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                0.8382,
                                //Constants.SubsystemConstants.TARGET_HEIGHT_METERS[0],
                                0,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));
                            }
    }

    */


	// Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return doneTraveling;
    }

    @Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
}