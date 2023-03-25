package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.NetworkTables;

public class AutoAlign extends CommandBase {

	
    private double desiredDistance;
    private double currentAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
	private double distanceTraveled;
	private double objectX;
    private double objectY;
    private String objectType;
    private double objectistance;
	private DriveBase drivebase;
	private NetworkTables networktables;
    private Arm arm;
    
    private double startTime;
    private double currentTime;
    
    private boolean isCentered;
	private double left, right;
	private double resultYaw;

	public AutoAlign(double SpeedInput, DriveBase passedDrivebase, NetworkTables passedNetworkTables) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);

		/*try {
			desiredDistance = networktables.getBallDistance();
		}
		catch (NullPointerException e)
		{
			desiredDistance = 0;
		}	*/

		autoDriveSpeed = SpeedInput;
		distanceTraveled = 0;
		drivebase = passedDrivebase;
		networktables = passedNetworkTables;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		
		doneTraveling = false;
		isCentered = false;
		distanceTraveled = 0;
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
		   //System.out.println("Turning right");
   
		 }
		   //Turning left
		 else if(resultYaw < (-1)){
		   left = -0.4;
		   right = 0.4;
		   //System.out.println("Turning left");
		 }
		   //Driving straight 
		 else{
			 //No joystick manipulation. 
		   left = 0.4;
		   right = 0.4;
		   //System.out.println("Moving straight");
   
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
