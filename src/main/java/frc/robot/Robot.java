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
import frc.robot.Subsystems.NetworkTables;
import frc.robot.Controls;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.StationPickup2;
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
private NetworkTables networkTables;
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



  

PhotonPipelineResult result;
Robot() {
  super(0.05);
}


@Override
public void robotInit() {
  
  robotContainer = new RobotContainer();
  
}

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
    drivebase.resetEncoders();
    drivebase.resetGyroAngle();
   
}

/**
 * This function is called periodically during autonomous.
 */
@Override
public void autonomousPeriodic() {
  feedWatchdogs();
  drivebase.updateOdometry();
  SmartDashboard.putNumber("Left Encoder", drivebase.getLeftEncoder());
  SmartDashboard.putNumber("Right Encoder", drivebase.getRightEncoder());

}

@Override
public void teleopInit() {
  

  if(autoSelected != null) {
    autoSelected.cancel();
  }
  ledSubsystem = robotContainer.getLedSubsystem();

  drivebase = robotContainer.getDriveBase();
  arm = robotContainer.getArm();
  claw = robotContainer.getClaw();
  
  drivebase.resetEncoders();
  drivebase.resetGyroAngle();

  lidar = robotContainer.getLidar();
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

  
  ledSubsystem = robotContainer.getLedSubsystem();

  feedWatchdogs();






    
if(arm.getArmMode()){
if(Controls.xbox_operator.getLeftY() < 0.05){
  arm.rotateArm(Controls.xbox_operator.getLeftY()*0.45, true);
}
else if(Controls.xbox_operator.getLeftY() > -0.05){
  arm.rotateArm(Controls.xbox_operator.getLeftY()*0.45, true);
}
else{
  arm.rotateArm(0, false);

}

}
else{
  if(Controls.xbox_operator.getLeftY() > 0.05){
    arm.rotateArm(-Controls.xbox_operator.getLeftY()*0.45, true);
  }
  else if(Controls.xbox_operator.getLeftY() < -0.05){
    arm.rotateArm(-Controls.xbox_operator.getLeftY()*0.45, true);
  }
  else{
    arm.rotateArm(0, false);
  
  }
}



if(Controls.xbox_operator.getRightX()*0.5 > 0.05){
  arm.translateArm(0.7, true);
}
else if(Controls.xbox_operator.getRightX()*0.5 < -0.05){
  arm.translateArm(-0.6,true);
}
else{
  arm.translateArm(0, false);

}




if (Controls.xbox_operator.getStartButton()){
  arm.resetArmEncoderDistance();
}




  drivebase.setCompressor(drivebase.getPressureStatus());
}
@Override
public void disabledPeriodic() {}

@Override
public void disabledInit() {}

@Override
public void testInit() {
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

