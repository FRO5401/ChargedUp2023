package frc.robot.Subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
     //PID stuff
  //private int loopIndex, slotIndex;
  private CANSparkMax armMotorLeft, armMotorRight;
  private CANSparkMax transMotor;
  private RelativeEncoder rotate_encoder, trans_encoder;
  private RelativeEncoder rotate_encoder_left, rotate_encoder_right;
  SparkMaxPIDController pidRotateMotorLeft;
  SparkMaxPIDController pidRotateMotorRight;
  SparkMaxPIDController pidTransMotor;

  DigitalInput minSwitch, maxSwitch;

  private double kFF = 1;
  private double kP = 23.183;
  private double kI = 0;//1;
  private double kD = 0.69334;//1.5784;
  private double kMinOutput;
  private double maxVel = 36.211; 
  private double minVel;
  private double maxAcc;
  private double allowedErr = 0.125;

  public Arm() {
    //armMotorLeft = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_RIGHT, MotorType.kBrushless);
    transMotor = new CANSparkMax(Constants.DriveConstants.TRANS_MOTOR, MotorType.kBrushless);

  
    //pidRotateMotorLeft = armMotorLeft.getPIDController();
    pidRotateMotorRight = armMotorRight.getPIDController();
    pidTransMotor = transMotor.getPIDController();

    //rotate_encoder_left = armMotorLeft.getEncoder();
    rotate_encoder_right = armMotorRight.getEncoder();

    trans_encoder = transMotor.getEncoder();

    /* 
    pidRotateMotorLeft.setP(0.022642);
    pidRotateMotorLeft.setI(0);
    pidRotateMotorLeft.setD(0.12823);
    */
    //pidTransMotor.setP(0.0151642);
    pidRotateMotorRight.setP(0.022642);
    pidRotateMotorRight.setI(0);
    pidRotateMotorRight.setD(0.12823);
    //pidTransMotor.setP(0.0151642);
   

    pidTransMotor.setP(0.0855);
    pidTransMotor.setI(0);
    pidTransMotor.setD(0.12823);
    
    
    //armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);




  }
  
  public void rotateArm(double armSpeed, boolean run){
    if(run){
      armMotorRight.set(armSpeed);
      //armMotorLeft.set(-armSpeed);


    }
    else{
      armMotorRight.set(0);
      //armMotorLeft.set(0);
    }

  }

  public void translateArm(double armSpeed, boolean run){
    if(run){
      transMotor.set(armSpeed);


    }
    else{
      transMotor.set(0);
    }
  }

  public void pidRotateArm(double position){
    //pidRotateMotorLeft.setReference(-43.99, ControlType.kPosition);
    //pidRotateMotorRight.setReference(-43.99, ControlType.kPosition);

    //setPIDPosition(pidRotateMotorLeft, rotate_encoder_left, ControlType.kPosition,  -position );
    setPIDPosition(pidRotateMotorRight, rotate_encoder_right, ControlType.kPosition,  position);

    //etPIDPosition(pidRotateMotorRight, rotate_encoder, ControlType.kPosition,  -43.99 );

    //setPIDPosition(pidRotateMotor, rotate_encoder, ControlType.kPosition,  -40.99, armMotorLeft);
    //setPIDPosition(pidRotateMotor, rotate_encoder, ControlType.kPosition,  setpoint );
      
    
  }

  public void setSlowPID(boolean mode){
    if(mode==true){
      /* 
    pidRotateMotorLeft.setP(0.018642);
    pidRotateMotorLeft.setI(0);
    pidRotateMotorLeft.setD(0.12823);
    */
    //pidTransMotor.setP(0.0151642);
    pidRotateMotorRight.setP(0.018642);
    pidRotateMotorRight.setI(0);
    pidRotateMotorRight.setD(0.12823);
    //pidTransMotor.setP(0.0151642);
   

    pidTransMotor.setP(0.0755);
    pidTransMotor.setI(0);
    pidTransMotor.setD(0.15823);
    }
    else{
      /* 
      pidRotateMotorLeft.setP(0.022642);
      pidRotateMotorLeft.setI(0);
      pidRotateMotorLeft.setD(0.12823);
      */
      //pidTransMotor.setP(0.0151642);
      pidRotateMotorRight.setP(0.022642);
      pidRotateMotorRight.setI(0);
      pidRotateMotorRight.setD(0.12823);
      //pidTransMotor.setP(0.0151642);
     
  
      pidTransMotor.setP(0.0855);
      pidTransMotor.setI(0);
      pidTransMotor.setD(0.12823);
    }
  }
  public void pidTranslateArm(double position){
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, 67.01, armMotorLeft );
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  167, transMotor );
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  167 );

    setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  position );
  }
  public void resetArmEncoderDistance(){
    trans_encoder.setPosition(0);
    //rotate_encoder_left.setPosition(0);
    rotate_encoder_right.setPosition(0);

  }

  public void getSwitches(){
    System.out.println("Min switch value: " + minSwitch.get());
    System.out.println("Max switch value: " + maxSwitch.get());

  }
  
  public void initPIDController(CANPIDController m_pidController, double kP, double kI, double kD, double kIz, double kMaxOutput){
    double p = 0, i = 0, d = 0, iz = 0, ff = 0, max = 0, min = 0, maxV = 0 , minV = 0, maxA = 0, allE = 0;
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  }

  public void setPIDPosition(CANPIDController pidTransMotor2, RelativeEncoder m_encoder,  ControlType kposition, double setPoint){    

      //motor.set(pidRotateMotor2.calculate(setPoint));
    
    //pidTransMotor2.setReference(setPoint, ControlType.kPosition);
    pidTransMotor2.setReference(setPoint, ControlType.kPosition);
  }
  public void printEncoderDistances(){
    //System.out.println("Rotational Encoder Left " + rotate_encoder_left.getPosition());
    System.out.println("Rotational Encoder Right " + rotate_encoder_right.getPosition());

    System.out.println("Translational Encoder" + trans_encoder.getPosition());
  }
  public void stationPickup(){
      pidRotateArm(15.5);
      pidTranslateArm(20.5);



  }
  public void groundPickup(){
      pidRotateArm(15.5);
      pidTranslateArm(20.5);

  }

  public void lowerNodePlace(){
      pidRotateArm(15.5);
      pidTranslateArm(20.5);


  }
  
  public void upperNodePlace(){
  
      pidRotateArm(15.5);
      pidTranslateArm(20.5);
  }

  public void cwRotate(){
    rotateArm(0.25, true);
  }
  public void ccwRotate(){
    rotateArm(-0.25, true);
  }
  public void extendOut(){
    translateArm(0.25, true);
  }
  public void extendIn(){
    translateArm(-0.25, true);
  }

public void resetArm(){
      pidRotateArm(0);
      pidTranslateArm(0);
  
  }


 
}
