// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import javax.sql.rowset.spi.TransactionalWriter;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.lang.Thread;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.ConeClaw;
import frc.robot.Commands.CubeClaw;
import frc.robot.Commands.GroundPickup;
import frc.robot.Commands.LowerNodePlace1;
import frc.robot.Commands.LowerNodePlaceB;
import frc.robot.Commands.OffClaw;
import frc.robot.Commands.StationPickup1;
import frc.robot.Commands.StationPickup2;
import frc.robot.Commands.UpperNodePlace;
//import frc.robot.Commands.OperatorControl;
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.ZeroGeneral;
import frc.robot.Commands.ZeroGround;
import frc.robot.Commands.gearShiftHigh;
import frc.robot.Commands.gearShiftLow;
import frc.robot.Commands.rotateArmPID;
//import frc.robot.Commands.rotateArmUp;
import frc.robot.Commands.translateIn;
import frc.robot.Commands.translateOut;
public class Robot extends TimedRobot {

private Command autoSelected;


private RobotContainer robotContainer;
private DriveBase drivebase;
private LEDSubsystem ledSubsystem;
private NetworkTables networkTables;
private Arm arm;
private Claw claw;
private Command xboxmove;
//private Command operator;










private boolean precision;
private double throttle = 0;
private double reverse = 0;
private boolean brake;
private double turn = 0;
private boolean rotate;
private double sensitivity = 0.8;
private double left = 0;
private double right  = 0;

double rotationLeft, rotationRight, tranLength;
private int dpad;
AddressableLED m_led;
AddressableLEDBuffer m_ledBuffer;

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
  //operator.schedule();
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
  //lidar.reportLidarDistance();
  //lidar.update();

  drivebase = robotContainer.getDriveBase();
  arm = robotContainer.getArm();
  claw = robotContainer.getClaw();
  networkTables = robotContainer.getNetworkTables();


 

  throttle = Controls.driver.getRightTriggerAxis();
  reverse = Controls.driver.getLeftTriggerAxis();
  turn = Controls.driver.getLeftX();
  precision = Controls.xbox_driver.getRightBumper();
  brake = Controls.xbox_driver.getLeftBumper();
  rotate = Controls.xbox_driver.getLeftStickButton();
  result = drivebase.getCamera().getLatestResult();

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
     drivebase.drive(left, right);
   
     //After speed manipulation, send to drivebase
    /*
     if(Controls.xbox_driver.getYButton()){
      var result = drivebase.getCamera().getLatestResult();

            if (result.hasTargets()) {

  
      if(result.getBestTarget().getYaw() > 4){
        left = 0.4;
        right = -0.4;
        System.out.println("Turning right");

      }
        //Turning left
      else if(result.getBestTarget().getYaw() < (-4)){
        left = -0.4;
        right = 0.4;
        System.out.println("Turning left");
      }
        //Driving straight 
      else{
          //No joystick manipulation. 
        left = 0.5;
        right = 0.5;
        System.out.println("Moving straight");

      }

    }
    else{
      left = 0.6;
        right = -0.6;
    }
  }

  drivebase.drive(left, right);
*/
  //new OperatorControl(arm, claw);
  //drivebase = robotContainer.getDriveBase();
  //arm = robotContainer.getArm();
  //result = drivebase.getCamera().getLatestResult();
  //System.out.println(result);

  //arm.printEncoderDistances();
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


if(Controls.xbox_operator.getRightBumper()){
  //arm.continuousPIDRotateArm("up");//arm.continuousPIDRotateArm("CW");//;translateArm(0.3*-1, Controls.operator.getXButton());
  arm.rotateArm(0.2*-1, true);
}
else if(Controls.xbox_operator.getLeftBumper()){
  //arm.continuousPIDRotateArm("down");
  arm.rotateArm(0.2, true);
}
else{
  arm.rotateArm(0, false);

}



if(Controls.xbox_operator.getXButton()){
  arm.translateArm(0.2*-1, true);// arm.continuousPIDRotateArm("CCW");//;translateArm(0.3*-1, Controls.operator.getXButton());
}
else if(Controls.xbox_operator.getBButton()){
  arm.translateArm(0.2,true);
}
else{
  arm.translateArm(0, false);

}




if (Controls.xbox_operator.getStartButton()){
  arm.resetArmEncoderDistance();

  //System.out.println("Reset");
  //robotContainer.drivebase.gearShift();
}

arm.updatePID();


/* 
if (Controls.operator.getBackButton()){
  arm.activateFrictionBrake();
  //System.out.println("Reset");
  //robotContainer.drivebase.gearShift();
}
*/

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
  case 0:
  //arm.pidRotateArm(60, 18);
  //arm.pidTranslateArm(10);
    rotationLeft = 60;
    rotationRight = 18;
    tranLength = 10;
    break;
  case 90:
  
    //arm.pidRotateArm(0, 0);
    //arm.pidTranslateArm(0);
    rotationLeft = 0;
    rotationRight = 0;
    tranLength = 0;
    break;
  case 180:
  //ground pickup
    
      //arm.pidRotateArm(60, 6);
      //arm.pidTranslateArm(10); 
    rotationLeft = 60;
    rotationRight = 6;
    tranLength = 10;
    break;
  //place lower node
  case 270:
    //arm.pidRotateArm(60, 20);
    //arm.pidTranslateArm(40);
    rotationLeft = 60;
    rotationRight = 20;
    tranLength  = 40;
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

  claw.reportLidarDistance();
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

