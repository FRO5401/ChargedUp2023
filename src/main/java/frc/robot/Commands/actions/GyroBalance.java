package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Controls;


public class GyroBalance extends CommandBase {
  /*** Variables ***/
    //Input Axes
    
    
  /*
  boolean speedConstant1;
  boolean speedConstant2;
  boolean speedConstant3;
 */
    //Instance Vars
  double speed;

  DriveBase drivebase;

  public GyroBalance(DriveBase m_drivebase, double m_speed) {
    drivebase = m_drivebase;
    speed = m_speed;
    
  }
  

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    drivebase.resetEncoders();
    drivebase.resetGyroAngle();

    //Printer.print("XboxMove");
  }

  @Override
  public void execute() {
    if(drivebase.getRoll() <= -10 || drivebase.getRoll() >= 10){
        drivebase.drive(speed *0.65 * -1 * (drivebase.getRoll()/(Math.abs(drivebase.getRoll()))), speed * 0.65 * -1* (drivebase.getRoll()/(Math.abs(drivebase.getRoll()))));
    }
    //System.out.println(drivebase.getPitch());
    addRequirements(drivebase);
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0,0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
    public boolean runsWhenDisabled() {
      return false;
  }
}