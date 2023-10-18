package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class AutoDriveFast extends CommandBase {

    private DriveBase drivebase;
	private double angle, desiredDistance, autoDriveSpeed, distanceTraveled; //Can declare variables next to each other
	private boolean doneTraveling;

	public AutoDriveFast(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
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
        //drivebase.gearShift("HIGH");
        drivebase.resetGyroAngle();
        distanceTraveled = 0;

    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        angle = drivebase.getGyro();
        distanceTraveled = Math.abs(drivebase.getPosition());
        if ((distanceTraveled < desiredDistance)) {
           if(distanceTraveled < desiredDistance*0.2){
            drivebase.autoDrive(autoDriveSpeed*0.6, autoDriveSpeed*0.6, angle);
            doneTraveling = false;
           }

            else if (distanceTraveled <  desiredDistance*0.8 && distanceTraveled > desiredDistance * 0.2){
            drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, angle);
            doneTraveling = false;
            }
            else if(distanceTraveled >  desiredDistance *0.7 && distanceTraveled < desiredDistance){
                drivebase.autoDrive(autoDriveSpeed*0.6, autoDriveSpeed*0.6, angle);
                doneTraveling = false;

            }





        }

            else{ //When leftDrive1 and rightDrive1 are zero

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