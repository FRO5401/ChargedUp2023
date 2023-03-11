package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Utilities.PowerManagement;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import java.util.ArrayList;
import java.util.Collection;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//TODO: Remember to bring gear shifter back
public class DriveBase extends SubsystemBase {
  private AHRS navxGyro;
  //true means facing front
  private boolean facingMode = true;
  PhotonCamera camera;

  // Configuring Motors
  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private CANSparkMax leftDrive3;

  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private CANSparkMax rightDrive3;

  private CANSparkMax armMotorLeft, armMotorRight;
  private CANSparkMax transMotor;

  DigitalInput minSwitch, maxSwitch;

  private CANPIDController pidTransMotor;

  SparkMaxPIDController pidRotateMotor;

  private RelativeEncoder rotate_encoder, trans_encoder;

  // Configuring Drives
  private MotorControllerGroup leftDrives;
  private MotorControllerGroup rightDrives;
  private DifferentialDrive ourDrive;
  private DifferentialDriveOdometry odometry;

  private boolean compressorState = false;

  //PID stuff
  //private int loopIndex, slotIndex;
  private double kFF = 1;
  private double kP = 23.183;
  private double kI = 0;//1;
  private double kD = 0.69334;//1.5784;
  private double kMinOutput;
  private double maxVel = 36.211; 
  private double minVel;
  private double maxAcc;
  private double allowedErr = 0.125;

  //private int iaccum = 0;

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
    //compressor.disable();
    compressor.enableDigital();
    //camera = new PhotonCamera("vision");
    camera = new PhotonCamera("NexiGo_N930AF_FHD_Webcam");


    pdp = new PowerDistribution();
    navxGyro = new AHRS(SPI.Port.kMXP);
    //camera = new PhotonCamera("vision");

  
  leftDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_1, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_2, MotorType.kBrushless);
   leftDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_LEFT_3, MotorType.kBrushless);
    rightDrive1 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_1, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_2, MotorType.kBrushless);
   rightDrive3 = new CANSparkMax(Constants.DriveConstants.DRIVE_MOTOR_RIGHT_3, MotorType.kBrushless);
   odometry = new DifferentialDriveOdometry(navxGyro.getRotation2d(), 0, 0);

    //armMotorLeft = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_LEFT, MotorType.kBrushless);
    //armMotorRight = new CANSparkMax(Constants.DriveConstants.ARM_MOTOR_RIGHT, MotorType.kBrushless);

    leftEncoders = new RelativeEncoder[3];
    rightEncoders = new RelativeEncoder[3];
    leftEncoders[0] = leftDrive2.getAlternateEncoder(4096);
    leftEncoders[1] = leftDrive3.getAlternateEncoder(4096);
    leftEncoders[2] = leftDrive3.getAlternateEncoder(4096);

    rightEncoders[0] = rightDrive2.getAlternateEncoder( 4096);
    rightEncoders[1] = rightDrive3.getAlternateEncoder( 4096);
    rightEncoders[2] = rightDrive2.getAlternateEncoder( 4096);

    rightDrive1.setSmartCurrentLimit(35, 15);
    rightDrive2.setSmartCurrentLimit(35, 15);
    rightDrive3.setSmartCurrentLimit(35, 15);

    leftDrive1.setSmartCurrentLimit(35, 15);

    leftDrive2.setSmartCurrentLimit(35, 15);
    leftDrive3.setSmartCurrentLimit(35, 15);

    //minSwitch = new DigitalInput(0);
    //maxSwitch = new DigitalInput(1);
   


    gearShifter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

   

    //transMotor = new CANSparkMax(Constants.DriveConstants.TRANS_MOTOR, MotorType.kBrushless);

    //pidRotateMotorLeft = armMotorLeft.getPIDController();
    //pidRotateMotorRight = armMotorRight.getPIDController();

    //pidTransMotor = transMotor.getPIDController();
  /* 
  pidRotateMotor = new ProfiledPIDController(
      0.025642, 0, 0.12823,
  new TrapezoidProfile.Constraints(52.316, 7));
  pidTransMotor = new ProfiledPIDController(
    0.021642, 0, 0.12823,
  new TrapezoidProfile.Constraints(52.316, 7));
*/
   
    /* 
    trans_encoder = transMotor.getEncoder();

    
    pidRotateMotorLeft.setP(0.022642);
    pidRotateMotorLeft.setI(0);
    pidRotateMotorLeft.setD(0.12823);
    //pidTransMotor.setP(0.0151642);
    pidRotateMotorRight.setP(0.022642);
    pidRotateMotorRight.setI(0);
    pidRotateMotorRight.setD(0.12823);
    //pidTransMotor.setP(0.0151642);
   

    pidTransMotor.setP(0.0855);
    pidTransMotor.setI(0);
    pidTransMotor.setD(0.12823);
    
    
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);

    //armMotorLeft.follow(armMotorRight);
  //armMotorLeft.setInverted(true);
  */

    powerManagement = new PowerManagement();
    leftDrives = new MotorControllerGroup( leftDrive1, leftDrive2, leftDrive3);
    rightDrives = new MotorControllerGroup(rightDrive1, rightDrive2, rightDrive3);
   // drivebaseMotors.add(leftDrive3);
   // drivebaseMotors.add(rightDrive3);
   ourDrive = new DifferentialDrive(leftDrives, rightDrives);
    
   leftDrives.setInverted(false);
   ourDrive.setExpiration(0.1);
   ourDrive.setMaxOutput(1.0);


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

  public void pidDrive(double position){
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition, 67.01, armMotorLeft );
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  167, transMotor );
    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  167 );

    //setPIDPosition(pidTransMotor, trans_encoder, ControlType.kPosition,  position );
  }

  /*
  public PhotonCamera getCamera(){
    return camera;
  }
  */


/* 
  public void getSwitches(){
    System.out.println("Min switch value: " + minSwitch.get());
    System.out.println("Max switch value: " + maxSwitch.get());

  }
  */
  public PhotonCamera getCamera(){
    return camera;
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
  
  public void drive(double left, double right) {
    ourDrive.tankDrive(left, right);
  }
  public double getGyro(){
    return navxGyro.getAngle();
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    //return new DifferentialDriveWheelSpeeds(leftDrive2.getEncoder().getVelocity(), rightDrive2.getEncoder().getVelocity());
    return new DifferentialDriveWheelSpeeds(0,0);

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

  public void compressorToggle(){
    compressor.disable();
  }
  /* 
  public CommandBase tankDriveCommand(double left, double right) {

    return run(() -> drive(left, right))
        .withName("tankDrive");
  }

  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              leftEncoders[0].setPosition(0);
              rightEncoders[0].setPosition(0);
              
            })
        .andThen(run(() -> tankDriveCommand(speed, speed)))
        .until(
            () ->
                Math.max(leftEncoders[0].getPosition(), rightEncoders[0].getPosition())
                    >= distanceMeters)
        .finallyDo(interrupted -> stopMotors());
  }
  */
}