package frc.robot.Subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import static frc.robot.Utilities.Tabs.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
     //PID stuff
  //private int loopIndex, slotIndex;
  private CANSparkMax armMotorLeft, armMotorRight;
  private CANSparkMax transMotor;
  private RelativeEncoder trans_encoder;
  private RelativeEncoder rotate_encoder_left, rotate_encoder_right;
  private SparkMaxPIDController pidRotateMotorLeft;
  private SparkMaxPIDController pidRotateMotorRight;
  private SparkMaxPIDController pidTransMotor;

  private Solenoid frictionBrake;


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

  private boolean armMode = true;


  public Arm() {
    armMotorLeft = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    armMotorRight = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_RIGHT, MotorType.kBrushless);
    transMotor = new CANSparkMax(Constants.DriveConstants.TRANS_MOTOR, MotorType.kBrushless);

    frictionBrake = new Solenoid(PneumaticsModuleType.CTREPCM, 6);



    pidRotateMotorLeft = armMotorLeft.getPIDController();
    pidRotateMotorRight = armMotorRight.getPIDController();
    pidTransMotor = transMotor.getPIDController();

    rotate_encoder_left = armMotorLeft.getEncoder();
    rotate_encoder_right = armMotorRight.getEncoder();

    trans_encoder = transMotor.getEncoder();

    
    pidRotateMotorLeft.setP(0.032642);
    pidRotateMotorLeft.setI(0);
    pidRotateMotorLeft.setD(0.12823);
    
  
    //pidTransMotor.setP(0.0151642);
    pidRotateMotorRight.setP(0.032823);
    pidRotateMotorRight.setI(0);
    pidRotateMotorRight.setD(0.12823);
    //pidTransMotor.setP(0.0151642);
   

    pidTransMotor.setP(0.042);
    pidTransMotor.setI(0);
    pidTransMotor.setD(0.12823);
    
    
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    transMotor.setIdleMode(IdleMode.kBrake);



  }
  
  public void rotateArm(double armSpeed, boolean run){
    if(run){ //Rotates arm in the same direction
      armMotorRight.set(armSpeed);
      armMotorLeft.set(-armSpeed);


    }
    else{ //Sets both motors to 0 stopping arm movement
      armMotorRight.set(0);
      armMotorLeft.set(0);
    }

  }

  public void translateArm(double armSpeed, boolean run){
    if(run){ //Extends the arm's reach.
      transMotor.set(armSpeed);
    }
    else{ //Stops extension of the arm.
      transMotor.set(0);
    }
  }

  public void pidRotateArm(double positionLeft, double positionRight){ //Rotates arm to specific set point
    
    
    setPIDPosition(pidRotateMotorLeft, rotate_encoder_left, ControlType.kPosition,  positionLeft );
    setPIDPosition(pidRotateMotorRight, rotate_encoder_right, ControlType.kPosition,  -positionRight);

      
    
  }

  public boolean getArmMode(){ 
    return armMode;
  }
  public boolean setArmMode(){ 
    armMode = !armMode;
    return armMode;
  }

  public boolean rotateLeftAtSetpoint(double setpoint){
    if(rotate_encoder_left.getPosition() < setpoint + 1  && rotate_encoder_left.getPosition() > setpoint - 1  ){
      return true;
    }
    else{
      return false;
    }
  }
  public boolean rotateRightAtSetpoint(double setpoint){
    if(rotate_encoder_right.getPosition() < setpoint + 1  && rotate_encoder_right.getPosition() > setpoint - 1  ){
      return true;
    }
    else{
      return false;
    }
  }
  public boolean transAtSetpoint(double setpoint){
    if(trans_encoder.getPosition() < setpoint + 1  && trans_encoder.getPosition() > setpoint - 1  ){
      return true;
    }
    else{
      return false;
    }
  }

  public void continuousPIDRotateArm( String direction){

    if(direction.equalsIgnoreCase("up")){

    
    setPIDPosition(pidRotateMotorRight, rotate_encoder_right, ControlType.kPosition,  rotate_encoder_right.getPosition() + 0.2);
    setPIDPosition(pidRotateMotorLeft, rotate_encoder_left, ControlType.kPosition,  rotate_encoder_left.getPosition() + 0.2);

    }
    else if (direction.equalsIgnoreCase("down")){
      setPIDPosition(pidRotateMotorRight, rotate_encoder_right, ControlType.kPosition,  rotate_encoder_right.getPosition() - 0.2);
      setPIDPosition(pidRotateMotorLeft, rotate_encoder_left, ControlType.kPosition,  rotate_encoder_left.getPosition() - 0.2);

    }

  }


  

  
  public void pidTranslateArm(double position){
    setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, position );
  }

  public void updatePID(){
    pidTranslateArm(trans_encoder.getPosition());
    pidRotateArm(rotate_encoder_left.getPosition(), rotate_encoder_right.getPosition());
  }


  public void continuousPIDTranslateArm(String direction){

    if(direction.equalsIgnoreCase("extend")){
      setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, (trans_encoder.getPosition() + 0.1)  );
      
    }
    else if(direction.equalsIgnoreCase("contract")){
      setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, (trans_encoder.getPosition() - 0.1)  );

    }
    else{
      setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, (trans_encoder.getPosition() )  );
    }
  }

  
  public void resetArmEncoderDistance(){
    trans_encoder.setPosition(0);
    rotate_encoder_left.setPosition(0);
    rotate_encoder_right.setPosition(0);
  }

  public void getSwitches(){
    //System.out.println("Min switch value: " + minSwitch.get());
    //System.out.println("Max switch value: " + maxSwitch.get());

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
    System.out.println("Rotational Encoder Right " + rotate_encoder_right.getPosition());

    //System.out.println("Translational Encoder" + trans_encoder.getPosition());
  }
  public double reportRotationsalEncoder(){
    return rotate_encoder_right.getPosition();
  }
  
  public void armShuffleboard(){
    rotLeftSpeedEntry = testingTab.add("Left Motor Speed",armMotorLeft.get()).getEntry();
    rotRightSpeedEntry = testingTab.add("Right Motor Speed",armMotorRight.get()).getEntry(); 
    rotLeftPositionEntry = testingTab.add("Left Motor Position",armMotorLeft.getEncoder().getPosition()).getEntry();     
    rotRightPositionEntry = testingTab.add("Right Motor Position",armMotorRight.getEncoder().getPosition()).getEntry();  
    transSpeedEntry = testingTab.add("Trans Motor Speed",transMotor.get()).getEntry();
    transPositionEntry = testingTab.add("Trans Motor Position",transMotor.getEncoder().getPosition()).getEntry();  

  }
 
}
