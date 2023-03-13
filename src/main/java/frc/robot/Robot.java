// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Utilities.Controls;
import frc.robot.subsystems.Drivebase;
import frc.robot.commands.Drivebase.XboxMove;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot { 
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  String trajectoryJSON = "paths.Unnamed.wpilib.json";
  Trajectory trajectory = new Trajectory();
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
   CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getDrivebase().resetEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  } 

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer = new RobotContainer();
  }



// values 
  double turn;
  double reverse;
  double throttle; 
  double left;
  double right;
  double percision;
  boolean rotate;
  boolean brake;
  @Override
  public void teleopPeriodic() {
    turn = Controls.driver.getLeftX();
    throttle = Controls.driver.getRightTriggerAxis();
    reverse = Controls.driver.getLeftTriggerAxis();
    rotate = Controls.driver.getLeftStickButton();
    brake = Controls.driver.getRightBumper();

    if(Controls.driver.getLeftBumper()){
      percision = Constants.xboxConstants.PERCISION;
    }
    else{
      percision = 1;
    }
 
    if(turn > Constants.xboxConstants.AXIS_THRESHOLD){
      //Makes left slow down by a factor of how far the axis is pushed. 
      left = (throttle - reverse) *(1 - turn) * percision;
      right = (throttle - reverse) * percision; 
   }

    //Turning left
    else if(turn < (-1 * Constants.xboxConstants.AXIS_THRESHOLD)){
      //Makes right speed up by a factor of how far the axis is pushed. 
      left = (throttle - reverse) * percision;
      right = (throttle - reverse) * (1 + turn) * percision;
   }

    //Driving straight 
    else{
      //No joystick manipulation. 
      left = (throttle - reverse) * percision;
      right = (throttle - reverse) * percision;
    }

    if(rotate){
     if(Math.abs(turn) > Constants.xboxConstants.AXIS_THRESHOLD){
        left = -turn;
        right = turn;
     } 
    }
    if(brake){
      left = 0;
      right = 0;
     }
    //shuffleboard tabs
  SmartDashboard.putNumber("Throttle",throttle);
  SmartDashboard.putNumber("Left Motor", left);
  SmartDashboard.putNumber("Right Motor", right);
    // setting drivebase speed 
    m_robotContainer.getDrivebase().drive(left, right);

}
  
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {

  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    
  }


  }


