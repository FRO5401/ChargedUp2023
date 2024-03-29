// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import javax.sql.rowset.spi.TransactionalWriter;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.lang.Thread;
import java.nio.file.Path;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.jni.*;

import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.Lidar;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.StationPickup2;
//import frc.robot.Commands.OperatorControl;
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.ZeroGround;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoAlign;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.CubeClaw;
import frc.robot.Commands.actions.GroundPickup;
import frc.robot.Commands.actions.LEDCommand;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.gearShiftHigh;
import frc.robot.Commands.actions.gearShiftLow;
import frc.robot.Commands.actions.translateIn;
import frc.robot.Commands.actions.translateOut;
import frc.robot.Commands.actions.LEDCommand.LEDPatterns;
import frc.robot.Commands.groups.LowerNodePlace1;
import frc.robot.Commands.groups.StationPickup1;
import frc.robot.Commands.groups.UpperNodePlace;
public class Robot extends TimedRobot {

private Command autoSelected;


private RobotContainer robotContainer;
private DriveBase drivebase;
private LEDSubsystem ledSubsystem;
private Arm arm;
private Claw claw;
private Command xboxmove;
private Command tempCommand;
private Lidar lidar;
//private Command operator;

private boolean precision;
private double throttle = 0;
private double reverse = 0;
private boolean brake;
private double turn = 0;
private boolean rotate;
private double sensitivity = 1.0;
private double left = 0;
private double right  = 0;
private double distance = 0;

double rotationLeft, rotationRight, tranLength;
private int dpad;
AddressableLED m_led;
AddressableLEDBuffer m_ledBuffer;

String trajectoryJSON = "paths.Unnamed.wpilib.json";
Trajectory trajectory = new Trajectory();



  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a
   * parameter. The device will be automatically initialized with default
   * parameters.
   */
  //private ColorSensorV3 m_colorSensor;

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
  //xboxmove = robotContainer.getXboxMove();
  //operator = robotContainer.getOperator();
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

    if (autoSelected != null) {
      autoSelected.schedule();
    }
    drivebase = robotContainer.getDriveBase();
    arm = robotContainer.getArm();
    claw = robotContainer.getClaw();
    lidar = robotContainer.getLidar();
    //networkTables = robotContainer.getNetworkTables();
    drivebase.resetEncoders();
    drivebase.resetGyroAngle();

}

/**
 * This function is called periodically during autonomous.
 */
@Override
public void autonomousPeriodic() {
  feedWatchdogs();
  //drivebase.printDriveBaseEncoderDistances();
  drivebase.updateOdometry();
  SmartDashboard.putNumber("Left Encoder", drivebase.getLeftEncoder());
  SmartDashboard.putNumber("Right Encoder", drivebase.getRightEncoder());

}

@Override
public void teleopInit() {
  //xboxmove.schedule();
  //operator.schedule();

  if(autoSelected != null) {
    autoSelected.cancel();
  }
  ledSubsystem = robotContainer.getLedSubsystem();

  drivebase = robotContainer.getDriveBase();
  arm = robotContainer.getArm();
  claw = robotContainer.getClaw();
  //networkTables = robotContainer.getNetworkTables();
  drivebase.resetEncoders();
  drivebase.resetGyroAngle();

  lidar = robotContainer.getLidar();
  //tempCommand = new LEDCommand(ledSubsystem,  LEDCommand.LEDPatterns.EveryOther, 0, 0, 0);
  tempCommand =  new LEDCommand(ledSubsystem,  LEDCommand.LEDPatterns.EveryOther, 255, 165, 0);

}

/**
 * This function is called periodically during operator control.
 */
@Override
public void teleopPeriodic() {
  lidar = robotContainer.getLidar();

  SmartDashboard.putNumber("Left Encoder", drivebase.getLeftEncoder());
  SmartDashboard.putNumber("Right Encoder", drivebase.getRightEncoder());
  SmartDashboard.putNumber("Lidar Distance", lidar.getDistance());
  SmartDashboard.putBoolean("Gear mode", drivebase.getGear());
  SmartDashboard.putBoolean("Pressure Status", drivebase.getPressureStatus());

  //System.out.println(lidar.getDistance());
  //distance = lidar.getDistance();
  ledSubsystem = robotContainer.getLedSubsystem();

  feedWatchdogs();

  //lidar.reportLidarDistance();
  //lidar.update();
  //drivebase.updateOdometry();

  /*
  throttle = Controls.driver.getRightTriggerAxis();
  reverse = Controls.driver.getLeftTriggerAxis();
  turn = Controls.driver.getLeftX();
  precision = Controls.xbox_driver.getRightBumper();
  brake = Controls.xbox_driver.getLeftBumper();
  rotate = Controls.xbox_driver.getLeftStickButton();




  //result = drivebase.getCamera().getLatestResult();

  //m_colorSensor = new ColorSensorV3(i2cPort);

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
     if(precision){
      sensitivity = 0.7;
    }
    else{
      sensitivity = 1.0;
    }

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
         left = Constants.ControlConstants.SPIN_SENSITIVITY;
         right = Constants.ControlConstants.SPIN_SENSITIVITY * -1;
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
     //drivebase.drive(left, right);
   /*
    if(Controls.xbox_driver.getAButton()){
      var result = drivebase.getCamera().getLatestResult();

            if (result.hasTargets()) {


      if(result.getBestTarget().getYaw() > 4){
        left = 0.6;
        right = -0.6;
        System.out.println("Turning right");

      }
        //Turning left
      else if(result.getBestTarget().getYaw() < (-4)){
        left = -0.6;
        right = 0.6;
        System.out.println("Turning left");
      }
        //Driving straight
      else{
          //No joystick manipulation.
        left = 0.6;
        right = 0.6;
        System.out.println("Moving straight");

      }

    }
    else{
      left = 0.6;
        right = -0.6;
    }
  }
*/
  //drivebase.drive(left, right);


  //new OperatorControl(arm, claw);
  //drivebase = robotContainer.getDriveBase();
  //arm = robotContainer.getArm();
  //result = drivebase.getCamera().getLatestResult();
  //System.out.println(result);

  //arm.printEncoderDistances();
  //drivebase.printDriveBaseEncoderDistances();

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
/*

if(Controls.operator.getRightTriggerAxis() > 0.1){
  arm.rotateArm(0.4*-1, Controls.operator.getXButton());
}
else if(Controls.operator.getLeftTriggerAxis() > 0.1){
  arm.rotateArm(0.4, Controls.operator.getBButton());

}
else{
  arm.rotateArm(0, false);
}
*/
if(arm.getArmMode()){
if(Controls.xbox_operator.getLeftY() < 0.05){
  //arm.continuousPIDRotateArm("up");//arm.continuousPIDRotateArm("CW");//;translateArm(0.3*-1, Controls.operator.getXButton());
  arm.rotateArm(Controls.xbox_operator.getLeftY()*0.45, true);
}
else if(Controls.xbox_operator.getLeftY() > -0.05){
  //arm.continuousPIDRotateArm("down");
  arm.rotateArm(Controls.xbox_operator.getLeftY()*0.45, true);
}
else{
  arm.rotateArm(0, false);

}

}
else{
  if(Controls.xbox_operator.getLeftY() > 0.05){
    //arm.continuousPIDRotateArm("up");//arm.continuousPIDRotateArm("CW");//;translateArm(0.3*-1, Controls.operator.getXButton());
    arm.rotateArm(-Controls.xbox_operator.getLeftY()*0.45, true);
  }
  else if(Controls.xbox_operator.getLeftY() < -0.05){
    //arm.continuousPIDRotateArm("down");
    arm.rotateArm(-Controls.xbox_operator.getLeftY()*0.45, true);
  }
  else{
    arm.rotateArm(0, false);

  }
}



if(Controls.xbox_operator.getRightX()*0.5 > 0.05){
  arm.translateArm(0.7, true);// arm.continuousPIDRotateArm("CCW");//;translateArm(0.3*-1, Controls.operator.getXButton());
}
else if(Controls.xbox_operator.getRightX()*0.5 < -0.05){
  arm.translateArm(-0.6,true);
}
else{
  arm.translateArm(0, false);

}




if (Controls.xbox_operator.getStartButton()){
  arm.resetArmEncoderDistance();

  //System.out.println("Reset");
  //robotContainer.drivebase.gearShift();
}

//arm.updatePID();


/*
if (Controls.xbox_driver.getBackButton()){
  drivebase.gearShift("LOW");
  //System.out.println("Reset");
  //robotContainer.drivebase.gearShift();
}
if (Controls.xbox_driver.getStartButton()){
  drivebase.gearShift("HIGH");
  //System.out.println("Reset");
  //robotContainer.drivebase.gearShift();
}
*/
/*
if(Controls.xbox_operator.getBButton()){
  if(lidar.getDistance() < 41 && claw.offMode() == true && lidar.getDistance() > 10){
    claw.toggleClaw("CONE");
  }
}
*/
//SmartDashboard.putNumber("Lidar Distance", lidar.getDistance());
/*
if(Controls.driver.getAButton()){
  System.out.println("Gyro Angle: " + drivebase.getGyro());
  if(drivebase.getGyro() <= -10 || drivebase.getGyro() >= 10){
    drivebase.drive(0.8 * -1 * (drivebase.getGyro()/(Math.abs(drivebase.getGyro()))), 0.8 * -1* (drivebase.getGyro()/(Math.abs(drivebase.getGyro()))));
}
}

*/
/*
dpad = Controls.xbox_driver.getPOV();
switch (dpad){
      //pickup charge station

  case 270:
  //arm.pidRotateArm(60, 18);
  //arm.pidTranslateArm(10);
    rotationLeft = 18;
    rotationRight = 18;
    arm.pidRotateArm(rotationLeft, rotationRight);
    break;


  case 90:

    //arm.pidRotateArm(0, 0);
    //arm.pidTranslateArm(0);
    rotationLeft = 0;
    rotationRight = 0;
    arm.pidRotateArm(rotationLeft, rotationRight);
    //tranLength = 0;
    break;
  case 180:
  //ground pickup

      //arm.pidRotateArm(60, 6);
      //arm.pidTranslateArm(10);
    rotationLeft = 5;
    rotationRight = 5;
    arm.pidRotateArm(rotationLeft, rotationRight);
    //tranLength = 10;
    break;
  //place lower node

  case 0:
    //arm.pidRotateArm(60, 20);
    //arm.pidTranslateArm(40);
    rotationLeft = 19;
    rotationRight = 19;
    arm.pidRotateArm(rotationLeft, rotationRight);
    //tranLength  = 40;
    break;


}
*/

//arm.pidRotateArm(rotationLeft, rotationRight);

//arm.pidTranslateArm(tranLength);


//Autonomous setpoints

//Controls.driver.rightTrigger(0.05).onTrue(new XboxMove(drivebase));
//Controls.driver.leftTrigger(0.05).onTrue(new XboxMove(drivebase));
//Controls.driver.leftStick().onTrue(new XboxMove(drivebase));

//Controls.operator.getRightBumper().
//Controls.operator.povLeft().onTrue(new (arm));
//Controls.operator.povLeft().onTrue(new UpperNodePlace(arm, claw));



//Controls.operator.a().onTrue();

  //two motors placement
  //robotContainer.drivebase.pidRotateArm(35.99);
  //robotContainer.drivebase.pidTranslateArm(64.01);
  //robotContainer.drivebase.pidTranslateArm(167.01);
  //two motors floor pickup 1 foot 4 inches aweay from pickup
  //robotContainer.drivebase.pidRotateArm(6.99);
  //robotContainer.drivebase.pidTranslateArm(90.01);
  //two motor station pickup 15.5 inches away from station
  //arm.pidRotateArm(0, 0);
  //arm.pidTranslateArm(0);
  drivebase.setCompressor(drivebase.getPressureStatus());

  //claw.reportLidarDistance();
}
@Override
public void disabledPeriodic() {}

@Override
public void disabledInit() {}

@Override
public void testInit() {
  //xboxmove.schedule();
  //operator.schedule();
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

