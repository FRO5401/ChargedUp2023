package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;

public class AutoTurn extends CommandBase {

    private DriveBase drivebase;
	private double desiredAngle;
	private double autoDriveSpeed;
	private boolean doneTraveling;
    private double turnStartTime, turnCurrentTime;
    private int count;

	public AutoTurn(double SpeedInput, double AngleInput, DriveBase passedDrivebase) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(drivebase);
        drivebase = passedDrivebase;
		desiredAngle = AngleInput;
		autoDriveSpeed = SpeedInput;
		doneTraveling = true;
        addRequirements(drivebase);

	}

	// Called just before this Command runs the first time
    @Override
    public void initialize() {
        drivebase.resetEncoders();
        drivebase.resetGyroAngle();
        doneTraveling = false;
        count = 0;
    }

	// Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        
        drivebase.autoTurn(autoDriveSpeed*0.9, desiredAngle);

        if(drivebase.getGyro() < desiredAngle+Constants.AutoConstants.ANGULAR_THRESHOLD && drivebase.getGyro() > desiredAngle-Constants.AutoConstants.ANGULAR_THRESHOLD)
            doneTraveling = true;
    }

    // Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
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