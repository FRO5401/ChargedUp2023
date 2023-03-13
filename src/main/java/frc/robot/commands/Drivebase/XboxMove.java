// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivebase;


import frc.robot.subsystems.Drivebase;


import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utilities.Controls;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */

//THIS FILE CURRENTLY DOES NOTHING 


public class XboxMove extends CommandBase {

  double Throttle; 
  boolean brake = false; 


  double turn;
  double reverse;
  double left;
  double right;
  /**   
   * Creates a new ExampleCommand.
   *  
   * @param subsystem The subsystem used by this command.
   */
  private final Drivebase drivebase;
  public XboxMove(Drivebase m_drivebase) {
    drivebase = m_drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    
    addRequirements(drivebase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // Shuffleboard.addEventMarker(getName(), null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.print(drivebase.Getaxis());
    turn = Controls.driver.getLeftX();
    Throttle = Controls.driver.getRightTriggerAxis();
    reverse = Controls.driver.getLeftTriggerAxis();

 
  if(turn > Constants.xboxConstants.AXIS_THRESHOLD){
      //Makes left slow down by a factor of how far the axis is pushed. 
    left = (Throttle - reverse);
    right = (Throttle - reverse) * (1 - turn);
  }
    //Turning left
  else if(turn < (-1 * Constants.xboxConstants.AXIS_THRESHOLD)){
      //Makes right speed up by a factor of how far the axis is pushed. 
    left = (Throttle - reverse) * (1 + turn);
    right = (Throttle - reverse);
  }
    //Driving straight 
  else{
      //No joystick manipulation. 
    left = (Throttle - reverse);
    right = (Throttle - reverse);
  }
  drivebase.drive(left, right);
}




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
