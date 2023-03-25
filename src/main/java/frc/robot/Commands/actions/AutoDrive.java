package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class AutoDrive extends CommandBase {

    private DriveBase drivebase;
	private double angle, desiredDistance, autoDriveSpeed, distanceTraveled; //Can declare variables next to each other
	private boolean doneTraveling;

	public AutoDrive(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
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
    @Override
    public void execute() {
        
        angle = drivebase.getGyro();
        distanceTraveled = Math.abs(drivebase.getPosition());
        if ((distanceTraveled < desiredDistance)) {
            drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
            doneTraveling = false;
            /* 
        } else if ((distanceTraveled > desiredDistance) && desiredDistance > 0) {
            drivebase.autoDrive(-autoDriveSpeed, -autoDriveSpeed, angle);
            doneTraveling = false;
        } else if ((distanceTraveled > desiredDistance) && desiredDistance < 0) {
            drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
            doneTraveling = false;
        } else if ((distanceTraveled < desiredDistance) && desiredDistance < 0) {
            drivebase.autoDrive(-autoDriveSpeed, -autoDriveSpeed, angle);    
            doneTraveling = false;
            */
        } else {
            drivebase.drive(0,0);
            doneTraveling = true;
        }
    }



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