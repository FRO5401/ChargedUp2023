// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.KinematicsConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Subsystems.DriveBase;

public class Drive extends CommandBase {

  DriveBase drivebase;

  XboxController driver;

  double xSpeed;
  double ySpeed;
  double turningSpeed;

  boolean FieldOriented;
  boolean resetGyro;

  ChassisSpeeds chassisSpeeds;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public Drive(DriveBase drivebase) {

    this.drivebase = drivebase;

    driver = RobotContainer.getDriverController();

    xLimiter = new SlewRateLimiter(ModuleConstants.MAX_SPEED);
    yLimiter = new SlewRateLimiter(ModuleConstants.MAX_SPEED);
    turningLimiter = new SlewRateLimiter(ModuleConstants.MAX_TURNING_SPEED);
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
    //get the axis values from the controller
    /*** CODE FOR CONTROLLER ***/
    /* 
    xSpeed = driver.getLeftX();
    ySpeed = driver.getLeftY();

    turningSpeed = driver.getRightX();
    */
    FieldOriented = driver.getBButton();
    resetGyro = driver.getAButton();

    /**** CODE FOR JOYSTICK ****/
       
    ySpeed = driver.getRawAxis(1);//driver.getLeftX();
    xSpeed = driver.getRawAxis(0);
    turningSpeed = driver.getRawAxis(2);
    

    if(resetGyro){
      drivebase.resetGyro();
    }

    //apply your deadzone area
    xSpeed = Math.abs(xSpeed) > ControllerConstants.AXIS_THRESHOLD ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > ControllerConstants.AXIS_THRESHOLD ? ySpeed : 0;
    turningSpeed = Math.abs(turningSpeed) > ControllerConstants.AXIS_THRESHOLD ? turningSpeed : 0;

    //Make it smooth for our drivers (We <3 our drivers here)
    xSpeed = xLimiter.calculate(xSpeed)  * ModuleConstants.MAX_TELEOP_SPEED ;
    ySpeed = yLimiter.calculate(ySpeed)  * ModuleConstants.MAX_TELEOP_SPEED ;
    turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.MAX_TELEOP_SPEED;

  if(!FieldOriented){
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, drivebase.getRotation2d());
  }    
  else{
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
  }

  SwerveModuleState[] moduleStates = KinematicsConstants.SWERVE_KINEMATIC.toSwerveModuleStates(chassisSpeeds);
  
  drivebase.setModuleStates(moduleStates);
  SmartDashboard.putNumber("Please", xSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
