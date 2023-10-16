package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveBase;

public class AutoAlign extends CommandBase {


	private boolean doneTraveling;
	private DriveBase drivebase;
	private double left, right;
	private double resultYaw;

	public AutoAlign(DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

		drivebase = passedDrivebase;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {

		doneTraveling = false;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		//After speed manipulation, send to drivebase

		 var result = drivebase.getCamera().getLatestResult();
		 if(result.getBestTarget() == null){
			result = drivebase.getCamera2().getLatestResult();

		 }
		 if(result.getBestTarget() == null){
			resultYaw = 0;
		 }
		 else{
		 resultYaw = result.getBestTarget().getYaw();
		 }

		if (result.hasTargets()) {
			if(resultYaw > 0 && drivebase.getCamera().getLatestResult() == null){
				resultYaw -= 14.0;
			}
			else{
				resultYaw -= 14.0;
			}


		 if(resultYaw > 1){
		   left = 0.4;
		   right = -0.4;


		 }
		   //Turning left
		 else if(resultYaw < (-1)){
		   left = -0.4;
		   right = 0.4;

		 }
		   //Driving straight
		 else{
			 //No joystick manipulation.
		   left = 0.4;
		   right = 0.4;

		 }

	   }



	 drivebase.drive(left,right);
	}
	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
	}

	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}
