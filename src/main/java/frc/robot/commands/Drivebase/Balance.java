// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;
import java.lang.Math;

public class Balance extends CommandBase {
  private Drivebase drivebase;
  /** Creates a new autoBalance. */
  public Balance(Drivebase m_drivebase) {
    drivebase = m_drivebase;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivebase.getAxis() > Constants.ANGLE_THRESHOLD || drivebase.getAxis() < -Constants.ANGLE_THRESHOLD){
      drivebase.drive(Constants.speedConstants.BALANCE_SPEED * -drivebase.getAxis(), Constants.speedConstants.BALANCE_SPEED * -drivebase.getAxis());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
