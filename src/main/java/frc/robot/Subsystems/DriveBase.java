package frc.robot.Subsystems;
import static frc.robot.Utilities.Tabs.*;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Commands.actions.gearShiftHigh;
import frc.robot.Utilities.PowerManagement;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.ArrayList;
import java.util.Collection;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementKindVisitor14;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  //true means facing front
  private boolean facingMode = true;
  PhotonCamera camera;
  PhotonCamera camera2;

  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private CANSparkMax leftDrive3;

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private CANSparkMax rightDrive3;


  private SparkMaxPIDController leftDrive1PidController;
  private SparkMaxPIDController leftDrive2PidController;
  private SparkMaxPIDController leftDrive3PidController;


  private SparkMaxPIDController rightDrive1PidController;
  private SparkMaxPIDController rightDrive2PidController;
  private SparkMaxPIDController rightDrive3PidController;


  DigitalInput minSwitch, maxSwitch;


  SparkMaxPIDController pidRotateMotor;

  private RelativeEncoder rotate_encoder, trans_encoder;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive ourDrive;
  private DifferentialDriveOdometry odometry;
  private int motorCurrent = 24;

  private boolean compressorState = false;

  //PID stuff
  //private int loopIndex, slotIndex;
  private double kP = 1.92;
  private double kI = 0;
  private double kD = 1;
  private double kMinOutput;
  private double maxVel = 3; 
  private double minVel;
  private double maxAcc;
  private double allowedErr = 0.125;

  // varibles for speed adjustment
  private double kEncoder = 0.01;
  double percentDifference;


  // Solenoid
  private Solenoid gearShifter;

  // Sensors
  private RelativeEncoder leftEncoders[];
  private RelativeEncoder rightEncoders[];
  public Compressor compressor;
  private PowerDistribution pdp;
  private PowerManagement powerManagement;
  Collection<CANSparkMax> drivebaseMotors;
  private double overCurrentTime;
  private boolean overCurrentFlag;
  SparkMaxPIDController pidRotateMotorLeft;
  PhotonTrackedTarget target;
  /**
   *
   */
  SparkMaxPIDController pidRotateMotorRight;
  RelativeEncoder rotate_encoder_left, rotate_encoder_right;

  public DriveBase() {

    //Instantating the physical parts on the drivebase
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    
    compressor.enableDigital();
    camera = new PhotonCamera("camera");
    camera2 = new PhotonCamera("camera2");

    pdp = new PowerDistribution();
    navxGyro = new AHRS(SPI.Port.kMXP);


  
    leftDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2, MotorType.kBrushless);
   leftDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_3, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2, MotorType.kBrushless);
   rightDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_3, MotorType.kBrushless);

    leftEncoders = new RelativeEncoder[3];
    rightEncoders = new RelativeEncoder[3];

    leftEncoders[0] = leftDrive1.getEncoder();
    leftEncoders[1] = leftDrive2.getEncoder();
    leftEncoders[2] = leftDrive3.getEncoder();

    rightEncoders[0] = rightDrive1.getEncoder();
    rightEncoders[1] = rightDrive2.getEncoder();
    rightEncoders[2] = rightDrive3.getEncoder();



    rightDrive1.setSmartCurrentLimit(motorCurrent, motorCurrent); //15, 10
    rightDrive2.setSmartCurrentLimit(motorCurrent, motorCurrent);
    rightDrive3.setSmartCurrentLimit(motorCurrent, motorCurrent);
    leftDrive1.setSmartCurrentLimit(motorCurrent,motorCurrent);
    leftDrive2.setSmartCurrentLimit(motorCurrent, motorCurrent);
    leftDrive3.setSmartCurrentLimit(motorCurrent, motorCurrent);

   


    gearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    drivebaseShuffleboard();


    leftDrive1PidController = leftDrive1.getPIDController();
    leftDrive2PidController = leftDrive2.getPIDController();
    leftDrive3PidController = leftDrive2.getPIDController();
    rightDrive1PidController = rightDrive1.getPIDController();
    rightDrive2PidController = rightDrive2.getPIDController();
    rightDrive3PidController = rightDrive2.getPIDController();


    powerManagement = new PowerManagement();
    leftDrives = new MotorControllerGroup( leftDrive1, leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);

   odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d(), leftEncoders[0].getPosition(), -rightEncoders[0].getPosition());

   ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    


   ourDrive.setExpiration(0.1);
   ourDrive.setMaxOutput(1.0);


   leftDrive1PidController.setP(kP);
   leftDrive1PidController.setI(kI);
   leftDrive1PidController.setD(kD);
   leftDrive2PidController.setP(kP);
   leftDrive2PidController.setI(kI);
   leftDrive2PidController.setD(kD);
   leftDrive3PidController.setP(kP);
   leftDrive3PidController.setI(kI);
   leftDrive3PidController.setD(kD);

   rightDrive1PidController.setP(kP);
   rightDrive1PidController.setI(kI);
   rightDrive1PidController.setD(kD);
   rightDrive2PidController.setP(kP);
   rightDrive2PidController.setI(kI);
   rightDrive2PidController.setD(kD);
   rightDrive3PidController.setP(kP);
   rightDrive3PidController.setI(kI);
   rightDrive3PidController.setD(kD);
    
  


   



  }



  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftDrives.setVoltage(leftVolts);
    rightDrives.setVoltage(-rightVolts);
    ourDrive.feed();
  }

  public void stopMotors(){
    leftDrives.set(0);
    rightDrives.set(0);
  }

  public void pidDrive(double velocityLeft, double velocityRight){
    setPIDVelocity(leftDrive1PidController, leftEncoders[0], ControlType.kSmartVelocity, velocityLeft );
    setPIDVelocity(leftDrive2PidController, leftEncoders[1], ControlType.kSmartVelocity, velocityLeft );
    setPIDVelocity(leftDrive3PidController, leftEncoders[2], ControlType.kSmartVelocity,  velocityLeft );


    setPIDVelocity(rightDrive1PidController, rightEncoders[0], ControlType.kSmartVelocity, velocityRight );
    setPIDVelocity(rightDrive2PidController, rightEncoders[1], ControlType.kSmartVelocity, velocityRight );
    setPIDVelocity(rightDrive3PidController,  rightEncoders[2], ControlType.kSmartVelocity,  velocityRight );
  }
  

public void switchVisionMode(int i){
  camera.setPipelineIndex(i);
} 
public void activateDriverMode(){
if(camera.getDriverMode() == true){
  camera.setDriverMode(false);

}
else{
  camera.setDriverMode(true);

}
}


  public PhotonCamera getCamera(){
    return camera;
  }
  public PhotonCamera getCamera2(){
    return camera2;
  }

 
  public float getPitch(){
    return navxGyro.getPitch();
  }

  public float getRoll(){
    
    return navxGyro.getRoll();

  }

  public void setPIDVelocity(CANPIDController pidTransMotor2, RelativeEncoder m_encoder,  ControlType kposition, double setPoint){    

    pidTransMotor2.setReference(setPoint, ControlType.kVelocity);
  }
  
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }
  public double getGyro(){
    return navxGyro.getAngle();
  }

  public PowerDistribution getPDP(){
    return pdp;
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition( navxGyro.getRotation2d(), 0, 0, pose);
  }
  
  public void gearShift(String mode){
    if(mode.equalsIgnoreCase("HIGH")){
      gearShifter.set(true);
    }
    else if(mode.equalsIgnoreCase("LOW")){
      gearShifter.set(false);
    }

  }

    public boolean getGear(){
      return gearShifter.get();
    }

  public double getPressure() { return compressor.getPressure();}

  public boolean getPressureStatus(){
    if(compressor.getPressure() <= 110.0){
      return true;
    }
    else{
      return false;
    }
  }

    public void compressorToggle(){
      compressorState = !compressorState;
      setCompressor(compressorState);
    }
  
    //Set the Compressor
    public void setCompressor(boolean state){
      if (state == false)
        compressor.disable();
      else
        compressor.enableDigital();  
    }

  public double getAverageMotorVelocity(){ return (Math.abs(leftEncoders[0].getVelocity())+Math.abs(rightEncoders[0].getVelocity()))/2; }
  public double getLeftVelocity() { return leftEncoders[0].getVelocity(); }
  public double getRightVelocity() { return rightEncoders[0].getVelocity(); }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), -getRightVelocity());

  }
    public void drivebaseShuffleboard(){
      //Graph conf
        
        //Testing Tab
        
        speedEntry = testingTab.add("Robot Speed",getAverageMotorVelocity()).getEntry();
        leftSpeedEntry = testingTab.add("Left Motor Speed",getLeftVelocity()).getEntry();
        rightSpeedEntry = testingTab.add("Right Motor Speed",getRightVelocity()).getEntry(); 
        leftPositionEntry = testingTab.add("Left Motor Position",leftEncoders[0].getPosition()).getEntry();     
        rightPositionEntry = testingTab.add("Right Motor Position",rightEncoders[0].getPosition()).getEntry();  
        rotationsEntry = testingTab.add("Gyro Rotations", getGyro()/360).getEntry();
        angleEntry = testingTab.add("Gyro Angle", getGyro()).getEntry();
        shifterEntry = testingTab.add("Solenoid Gear", getGear()).getEntry();
        pressureEntry = testingTab.add("Pressure ", getPressureStatus()).getEntry();
    
  }

  public double getPosition(){
    return leftEncoders[2].getPosition();
  }
  public void updateOdometry(){
    odometry.update(navxGyro.getRotation2d(), leftEncoders[1].getPosition(), rightEncoders[1].getPosition());
}
    public void resetEncoders() {
        leftEncoders[0].setPosition(0);
        leftEncoders[1].setPosition(0);
        leftEncoders[2].setPosition(0);
        rightEncoders[0].setPosition(0);
        rightEncoders[1].setPosition(0);
        rightEncoders[2].setPosition(0);

      
    }

    public void autoTurn(double speed, double angle) {
      double gyroAngle = getGyro();
      if (gyroAngle > (angle))
        drive(-speed, speed);
      else if (gyroAngle < (angle))
        drive(speed, -speed);
      else 
        drive(0, 0);
    }
// method used to fix the drift if the speed is wrong
    public double getAdjustedSpeed(double num, double dom){
      percentDifference = 1 - (kEncoder * (1 - (dom / num)));
      return percentDifference; 
    }

    public void autoDrive(double left, double right, double angle) {
     
      if (left > 0 && right > 0){ //driving forwards
        drive(
          angle > 0.1 ? left : left*1.01, 
          angle < -0.1 ? right : right*1.01 
        );
      }
      else if (left < 0 && right < 0){ //driving backwards
        drive(
          angle < -0.1 ? left : left*1.01,
          angle > 0.1 ? right : right*1.01 
        );
      }
      
    }

    public void resetGyroAngle() {
      navxGyro.reset();
    }
    
    public void smoothStop(){
      for(int i = 0; i < 1000; i++){
        leftDrives.set(-0.3);
        rightDrives.set(-0.3);

      }
      leftDrives.set(0);
      rightDrives.set(0);
    }
    public double getRightEncoder(){
      return rightEncoders[0].getPosition();
    }
    public double getLeftEncoder(){
      return leftEncoders[0].getPosition();
    }
}