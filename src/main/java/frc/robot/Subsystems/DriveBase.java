// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */
  AHRS Gyro = new AHRS(I2C.Port.kMXP);
  CANSparkMax leftDrive1; 
  CANSparkMax leftDrive2;  
  //CANSparkMax leftDrive3;
  CANSparkMax rightDrive1; 
  CANSparkMax rightDrive2;
  //CANSparkMax rightDrive3;
  MotorControllerGroup leftDrives;  
  MotorControllerGroup rightDrives;
  private static DifferentialDrive ourDrive;
  Encoder rightEncoder;
  Encoder leftEncoder;
  private final DifferentialDriveOdometry m_odometry;
  public Drivebase() {

    leftDrive1 = new CANSparkMax(Constants.motorConstants.MOTER1, MotorType.kBrushed);
    leftDrive2 = new CANSparkMax(Constants.motorConstants.MOTER2, MotorType.kBrushed); 
    //leftDrive3 = new CANSparkMax(Constants.motorConstants.MOTER3, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.motorConstants.MOTER3, MotorType.kBrushed); 
    rightDrive2 = new CANSparkMax(Constants.motorConstants.MOTER4, MotorType.kBrushed);
    //rightDrive3 = new CANSparkMax(Constants.motorConstants.MOTER6, MotorType.kBrushless);\
    leftEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k1X);
    rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k1X);
    m_odometry = new DifferentialDriveOdometry(Gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());
    leftDrives = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2);
    ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    
    rightDrives.setInverted(true);

    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.setMinRate(10);
    leftEncoder.setDistancePerPulse(0.1);
    rightEncoder.setDistancePerPulse(0.1);  
  }

  public double getLeftEncoder(){
    double rate = leftEncoder.getRate();
    return rate;
  }
  public double getRightEncoder(){
    return rightEncoder.getRate();
  }

  public void drive(double left, double right){
    ourDrive.tankDrive(left, right);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(rightVolts);
    ourDrive.feed();
  }

  public double getAxis(){
    return Gyro.getAngle();
  }
  public void resetGyro(){
    Gyro.reset();
  }
  public double getHeading() {
    return Gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -Gyro.getRate();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();    
  }
  


  public double getAverageEncoderDistance() {
      return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoder(), getRightEncoder());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left1 Motor Output", leftDrive1.getOutputCurrent());
    SmartDashboard.putNumber("Left2 Motor Output", leftDrive2.getOutputCurrent());
    SmartDashboard.putNumber("Right1 Motor Output", rightDrive1.getOutputCurrent());
    SmartDashboard.putNumber("Right2 Motor Output", rightDrive2.getOutputCurrent());
    SmartDashboard.putNumber("Left1 Id", leftDrive1.getDeviceId());
    SmartDashboard.putNumber("Left2 Id", leftDrive2.getDeviceId());
    SmartDashboard.putNumber("Right1 Id", rightDrive1.getDeviceId());
    SmartDashboard.putNumber("Right2 Id", rightDrive2.getDeviceId());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    m_odometry.update(Gyro.getRotation2d(), getLeftEncoder(), getRightEncoder());
  }
}

