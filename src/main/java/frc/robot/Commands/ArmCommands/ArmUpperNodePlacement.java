package frc.robot.Commands.ArmCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Controls;


public class ArmUpperNodePlacement extends CommandBase {
  /*** Variables ***/
    //Input Axes
    
    
  /*
  boolean speedConstant1;
  boolean speedConstant2;
  boolean speedConstant3;
 */
    //Instance Vars
  double speed;

  Arm arm;


  public ArmUpperNodePlacement(Arm m_arm, double m_speed) {
    arm = m_arm;
    speed = m_speed;
    
  }
  

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
   
    //Printer.print("XboxMove");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    arm.upperNodePlace(); 
  }

  @Override
  public void end(boolean interrupted) {
    
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