package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        /** 
		ballX = networktables.getBallXValue();
		ballY = networktables.getBallYValue();

		currentTime = Timer.getMatchTime();
		double timeElapsed = startTime - currentTime;
        SmartDashboard.putNumber("Time elapsed", timeElapsed);
		radius = networktables.getBallRadius();

		if(isCentered == false){
			isCentered = networktables.checkCentered("BALL");
			currentAngle = drivebase.getGyroYaw();

		}
		
		if(ballX == 0 && ballY == 0){ //If no ball is recognized, scan area
			isCentered = false;
			if(timeElapsed >= 3){//If no ball has been found after 3 seconds, go back to original angle and stop
				if(drivebase.getGyro() > (drivebase.getGyro() % 366)){
					drivebase.drive(autoDriveSpeed, autoDriveSpeed);
				}
				else if(drivebase.getGyro() < (drivebase.getGyro() % 366)){
					drivebase.drive(autoDriveSpeed, autoDriveSpeed);
				}
				else{
					drivebase.drive(0,0);
					doneTraveling = true;
					drivebase.resetGyroAngle();
				}	
			}
			else if((timeElapsed) < 3){
				drivebase.drive(0.2, 0.2);
			}
		}
		else{ //If ball is recognized drive towards it and infeed
		    if(isCentered == true) { //Once recognized ball is straight ahead, drive towards it based off of received distance
				infeed.run("START");

				if(radius < 200){
					drivebase.autoDrive(autoDriveSpeed, autoDriveSpeed, drivebase.getGyro());
				}
				else{
		    	    drivebase.drive(0,0);
					doneTraveling = true;
				}
            }
    	    else { //Turn until the ball that is recognized is straight ahead
			    if(((currentAngle+90)*3.56) < ballX){
				    drivebase.autoTurn(autoDriveSpeed, currentAngle);
					System.out.println(isCentered);
			    }
        	    else if(((currentAngle+90)*3.56) > ballX){
				    drivebase.autoTurn(autoDriveSpeed, currentAngle);
					System.out.println(isCentered);
				}
				else {
					drivebase.drive(0, 0);
				}
            }
        }
        **/
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return doneTraveling;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		drivebase.drive(0,0);
		// System.out.println("Angle when EXITING DriveShift:" +
		// drivebase.getGyroAngle());
	}

	@Override
  	public boolean runsWhenDisabled() {
    	return false;
  	}
	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
}
