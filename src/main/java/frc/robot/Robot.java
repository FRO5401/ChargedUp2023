// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.revrobotics.jni.*;

import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Controls;
import frc.robot.Commands.MiscCommands.XboxMove;
public class Robot extends TimedRobot {

private Command autoSelected;


private RobotContainer robotContainer;
private DriveBase drivebase;
//private Command arm;
private Command xboxmove;
private Command operator;









private boolean precision = false;
private double throttle = 0;
private double reverse = 0;
private boolean brake = false;
private double turn = 0;
private boolean rotate  = false;
private double sensitivity = 0.8;
private double left = 0;
private double right  = 0;
private int dpad;
PhotonPipelineResult result;
Robot() {
  super(0.05);
}

/**
 * This function is run when the robot is first started up and should be used
 * for any initialization code.
 */
@Override
public void robotInit() {
  
  robotContainer = new RobotContainer();
  xboxmove = robotContainer.getXboxMove();
  operator = robotContainer.getOperator();

}

/**
 * This function is called every robot packet, no matter the mode. Use this for
 * items like diagnostics that you want ran during disabled, autonomous,
 * teleoperated and test.
 *
 * <p>
 * This runs after the mode specific periodic functions, but before LiveWindow
 * and SmartDashboard integrated updating.
 */
@Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
  
}

@Override
public void autonomousInit() {
  CommandScheduler.getInstance().cancelAll();
    autoSelected = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoSelected != null) {
      autoSelected.schedule();
    }
}

/**
 * This function is called periodically during autonomous.
 */
@Override
public void autonomousPeriodic() {
  feedWatchdogs();

}

@Override
public void teleopInit() {
  //xboxmove.schedule();
  operator.schedule();
  if(autoSelected != null) {
    autoSelected.cancel();
  }

  
}

/**
 * This function is called periodically during operator control.
 */
@Override
public void teleopPeriodic() {
  feedWatchdogs();
  drivebase = robotContainer.getDriveBase();

  throttle = Controls.driver.getRightTriggerAxis();
   reverse = Controls.driver.getLeftTriggerAxis();
   turn = Controls.driver.getLeftX();
   precision = Controls.driver.getRightBumper();
   brake = Controls.driver.getLeftBumper();
  rotate = Controls.driver.getLeftStickButton();
  result = drivebase.getCamera().getLatestResult();

/*

   System.out.println(throttle);
   System.out.println(reverse);
   System.out.println(turn);
   System.out.println(precision);
   System.out.println(brake);
   System.out.println(rotate);
*/


     //Braking
      /*** Precision ***/
     //Hold for Precision Speed

   /*** Driving ***/
     //Braking
   if(brake){
     //robotContainer.drivebase.stopMotors();
     left = 0;
     right = 0;
   }   
       //Pirouetting (Turn in place). 
     if(rotate){
         //If the joystick is pushed passed the threshold. 
       if(Math.abs(turn) > Constants.ControlConstants.AXIS_THRESHOLD){
           //Sets it to spin the desired direction.
         left = Constants.ControlConstants.SPIN_SENSITIVITY * turn;
         right = Constants.ControlConstants.SPIN_SENSITIVITY * (turn * -1);
       }
         //If its not past the threshold stop spinning
       else if(Math.abs(turn) < Constants.ControlConstants.AXIS_THRESHOLD){
         left = 0;
         right = 0;
       }
     }
       //Not pirouetting (Not turning in place).
     else{
         //Turning right
       if(turn > Constants.ControlConstants.AXIS_THRESHOLD){
           //Makes left slow down by a factor of how far the axis is pushed. 
         left = (throttle - reverse) * sensitivity;
         right = (throttle - reverse) * sensitivity * (1 - turn);
       }
         //Turning left
       else if(turn < (-1 * Constants.ControlConstants.AXIS_THRESHOLD)){
           //Makes right speed up by a factor of how far the axis is pushed. 
         left = (throttle - reverse) * sensitivity  * (1 + turn);
         right = (throttle - reverse) * sensitivity;
       }
         //Driving straight 
       else{
           //No joystick manipulation. 
         left = (throttle - reverse) * sensitivity;
         right = (throttle - reverse) * sensitivity;
       }
     }
   
     //After speed manipulation, send to drivebase
    
     if(Controls.driver.getYButton()){
      var result = drivebase.getCamera().getLatestResult();

            if (result.hasTargets()) {

    
      if(result.getBestTarget().getYaw() > 2){
        left = -0.8;
        right = 0.8;
      }
        //Turning left
      else if(result.getBestTarget().getYaw() < (-2)){
        left = 0.8;
        right = -0.8;
      }
        //Driving straight 
      else{
          //No joystick manipulation. 
        left = -0.7;
        right = -0.7;
      }

    }
    else{
      left = 0.6;
        right = -0.6;
    }
  }

  drivebase.drive(left, right);
  //drivebase = robotContainer.getDriveBase();
  //arm = robotContainer.getArm();
  //result = drivebase.getCamera().getLatestResult();
  //System.out.println(result);

    //robotContainer.drivebase.printEncoderDistances();
    //robotContainer.drivebase.getSwitches();

   //throttle = Controls.xboxAxis(Controls.driver, "RT").getAxis();
   /*
   throttle = Controls.driver.getRightTriggerAxis();
   reverse = Controls.driver.getLeftTriggerAxis();
   turn = Controls.driver.getLeftX();
   precision = Controls.driver.getRightBumper();
   brake = Controls.driver.getLeftBumper();
  rotate = Controls.driver.getLeftStickButton();

*/

/*
   System.out.println(throttle);
   System.out.println(reverse);
   System.out.println(turn);
   System.out.println(precision);
   System.out.println(brake);
   System.out.println(rotate);
*/


     //Braking
      /*** Precision ***/
     //Hold for Precision Speed

   /*** Driving ***/
     //Braking
     /* 
   if(brake){
     //Robot.drivebase.stopMotors();
     left = 0;
     right = 0;
   }   
       //Pirouetting (Turn in place). 
     if(rotate){
      
         //If the joystick is pushed passed the threshold. 
       if(Math.abs(turn) > Constants.ControlConstants.AXIS_THRESHOLD){
           //Sets it to spin the desired direction.
         left = Constants.ControlConstants.SPIN_SENSITIVITY * turn;
         right = Constants.ControlConstants.SPIN_SENSITIVITY * (turn * -1);
       }
         //If its not past the threshold stop spinning
       else if(Math.abs(turn) < Constants.ControlConstants.AXIS_THRESHOLD){
         left = 0;
         right = 0;
       }
     }
       //Not pirouetting (Not turning in place).
     else{
         //Turning right
       if(turn > Constants.ControlConstants.AXIS_THRESHOLD){
           //Makes left slow down by a factor of how far the axis is pushed. 
         left = (throttle - reverse) * sensitivity;
         right = (throttle - reverse) * sensitivity * (1 - turn);
       }
         //Turning left
       else if(turn < (-1 * Constants.ControlConstants.AXIS_THRESHOLD)){
           //Makes right speed up by a factor of how far the axis is pushed. 
         left = (throttle - reverse) * sensitivity  * (1 + turn);
         right = (throttle - reverse) * sensitivity;
       }
         //Driving straight 
       else{
           //No joystick manipulation. 
         left = (throttle - reverse) * sensitivity;
         right = (throttle - reverse) * sensitivity;
       }
     }
     robotContainer.drivebase.drive(left, right);

   
     //After speed manipulation, send to drivebase
      /*
     if(Controls.driver.getYButton()){
      var result = drivebase.getCamera().getLatestResult();

            if (result.hasTargets()) {

    
      if(result.getBestTarget().getYaw() > 2){
        left = -0.8;
        right = 0.8;
      }
        //Turning left
      else if(result.getBestTarget().getYaw() < (-2)){
        left = 0.8;
        right = -0.8;
      }
        //Driving straight 
      else{
          //No joystick manipulation. 
        left = -0.7;
        right = -0.7;
      }
      robotContainer.drivebase.drive(left, right);

    }
    else{
      left = 0.6;
        right = -0.6;
    }
    robotContainer.drivebase.drive(left, right);

  }
    else{
      System.out.println("blingo wingo");

     robotContainer.drivebase.drive(left, right);
    }
   //robotContainer.drivebase.drive(Controls.driver.getLeftTriggerAxis(), Controls.driver.getRightTriggerAxis());
 
  
if(Controls.driver.getXButton()){
  robotContainer.arm.rotateArm(0.25*-1, Controls.driver.getXButton());
}
else if(Controls.driver.getBButton()){
  robotContainer.arm.rotateArm(0.25, Controls.driver.getBButton());

}
else{
  robotContainer.arm.rotateArm(0, Controls.driver.getXButton() || Controls.driver.getBButton());
}


if(Controls.driver.getRightBumper()){
  robotContainer.arm.translateArm(0.4*-1, Controls.driver.getRightBumper());
}
else if(Controls.driver.getLeftBumper()){
  robotContainer.arm.translateArm(0.4, Controls.driver.getLeftBumper());

}
else{
  robotContainer.arm.translateArm(0, Controls.driver.getLeftBumper() || Controls.driver.getRightBumper());
}

*/
/* 

if (Controls.driver.getStartButton()){
  //robotContainer.arm.resetArmEncoderDistance();
  //System.out.println("Reset");
  robotContainer.drivebase.gearShift();
}

if(Controls.driver.getAButton()){
  System.out.println("Gyro Angle: " + drivebase.getGyro());
  if(drivebase.getGyro() <= -10 || drivebase.getGyro() >= 10){
    drivebase.drive(0.8 * -1 * (drivebase.getGyro()/(Math.abs(drivebase.getGyro()))), 0.8 * -1* (drivebase.getGyro()/(Math.abs(drivebase.getGyro()))));
}
}
*/


/* 

dpad = Controls.driver.getPOV();
switch (dpad){
  case 0:
    robotContainer.arm.pidRotateArm(15.5);
    robotContainer.arm.pidTranslateArm(20.5);
    break;
  case 90:
    robotContainer.arm.pidRotateArm(32.01);
    robotContainer.arm.pidTranslateArm(167.01);
    break;
  case 180:
    robotContainer.arm.pidRotateArm(4.99);
    robotContainer.arm.pidTranslateArm(55.01);
    break;
  case 270:
    robotContainer.arm.pidRotateArm(35.99);
    robotContainer.arm.pidTranslateArm(64.01);
    break;
}

if(Controls.driver.getAButton()){
  
  //two motors placement
  //robotContainer.drivebase.pidRotateArm(35.99);
  //robotContainer.drivebase.pidTranslateArm(64.01);
  //robotContainer.drivebase.pidTranslateArm(167.01);
  //two motors floor pickup 1 foot 4 inches aweay from pickup
  //robotContainer.drivebase.pidRotateArm(6.99);
  //robotContainer.drivebase.pidTranslateArm(90.01);
  //two motor station pickup 15.5 inches away from station
  robotContainer.arm.setSlowPID(Controls.driver.getAButton());
  robotContainer.arm.pidRotateArm(0);
  robotContainer.arm.pidTranslateArm(0);
}
else{
  robotContainer.arm.setSlowPID(false);

}
}
*/
}

@Override
public void disabledPeriodic() {}

@Override
public void disabledInit() {}

@Override
public void testInit() {
  xboxmove.schedule();
  operator.schedule();
  CommandScheduler.getInstance().cancelAll();

}


@Override
public void testPeriodic() {
  feedWatchdogs();
}

private void feedWatchdogs(){
CANSparkMaxJNI.c_SparkMax_SetEnable(true); 
}

}

