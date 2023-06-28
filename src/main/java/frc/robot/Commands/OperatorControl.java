package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

import frc.robot.Subsystems.DriveBase;
import frc.robot.Controls;


public class OperatorControl extends CommandBase {
  /*** Variables ***/
    //Input Axes
    
    public final CommandXboxController operator = new CommandXboxController(Constants.ControlConstants.XBOX_CONTROLLER_DRIVER);
    Claw claw;
    Arm arm;
    boolean clawState;
    int dpad;
    
  public OperatorControl(Arm m_arm, Claw m_Claw) {
    arm = m_arm;
    claw = m_Claw;
    addRequirements(arm);
    addRequirements(claw);

  }
  

  // Called just before this Command runs the first time
  @Override
  public void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      
       
         
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