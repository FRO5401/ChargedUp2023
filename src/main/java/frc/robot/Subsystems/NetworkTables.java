package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class NetworkTables extends SubsystemBase {
  
    //public static NetworkTableEntry conesCXShuffleboard, conesCYShuffleboard, conesCXShuffleboard, conesCYShuffleboard;
    public static NetworkTableEntry targetXShuffleboard, targetYShuffleboard, targetDShuffleboard;
    public static NetworkTableEntry robotXShuffleboard, robotYShuffleboard, robotDShuffleboard;

  NetworkTable visionTable;
  NetworkTableEntry conesCXEntry, conesCYEntry, cubesCXEntry, cubesCYEntry, 
  redRobotCXEntry, redRobotCYEntry, blueRobotCXEntry, blueRobotCYEntry, objectDistanceEntry, robotDistanceEntry;
  NetworkTableInstance inst;

  private double conesCX, conesCY, cubesCX, cubesCY, 
  redRobotCX, redRobotCY, blueRobotCX, blueRobotCY, object_distance, robot_distance;


  public NetworkTables() {
    inst = NetworkTableInstance.getDefault();
    visionTable = inst.getTable("vision");

    conesCXEntry = visionTable.getEntry("conesCX");
    conesCYEntry = visionTable.getEntry("conesCY");
    cubesCXEntry = visionTable.getEntry("cubesCX");
    cubesCYEntry = visionTable.getEntry("cubesCY");
    redRobotCXEntry = visionTable.getEntry("red_robotCX");
    redRobotCYEntry = visionTable.getEntry("red_robotCY");
    blueRobotCXEntry = visionTable.getEntry("blue_robotCX");
    blueRobotCYEntry = visionTable.getEntry("blue_robotCY");
    objectDistanceEntry = visionTable.getEntry("object_distance");
    robotDistanceEntry = visionTable.getEntry("robotDistance");

    conesCX= 0; conesCY= 0; cubesCX= 0; cubesCY= 0; 
  redRobotCX= 0; redRobotCY= 0; blueRobotCX= 0; blueRobotCY= 0; object_distance= 0; robot_distance = 0;
    

    inst.setServer("Team5401", 8000); // where TEAM=190= 0; 294, etc, or use inst.
    inst.startDSClient();

    //networkTablesShuffleboard();
  }

  @Override
  public void periodic() {
    updateValue();
    //reportValues();
    //odometry.update(navxGyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void updateValue() {
     // recommended if running on DS computer; this gets the robot

    conesCX = conesCXEntry.getDouble((conesCX != 0) ? conesCX : 0);
    conesCY = conesCYEntry.getDouble((conesCY != 0) ? conesCY : 0);
    cubesCX = cubesCXEntry.getDouble((cubesCX != 0) ? cubesCX : 0);
    cubesCY = cubesCYEntry.getDouble((cubesCY != 0) ? cubesCY : 0);
    redRobotCX = redRobotCXEntry.getDouble((redRobotCX != 0) ? redRobotCX : 0);
    redRobotCY = redRobotCYEntry.getDouble((redRobotCY != 0) ? redRobotCY : 0);
    blueRobotCX = blueRobotCXEntry.getDouble((blueRobotCX != 0) ? blueRobotCX : 0);
    blueRobotCY = blueRobotCYEntry.getDouble((blueRobotCY != 0) ? blueRobotCY : 0);
    object_distance = objectDistanceEntry.getDouble((object_distance != 0) ? object_distance : 0);
    robot_distance = robotDistanceEntry.getDouble((object_distance != 0) ? object_distance : 0);

    //mode = (int)shooterVModeEntry.getDouble(mode);
    //powerPortX = powerPortXEntry.getDouble(0.0);
    //powerPortY = powerPortYEntry.getDouble(0.0);
    //System.out.println("The Ball coordinates are: " + "X: " + ballX + " Y: " + ballY);
    //System.out.println("The Ball is " + ballDistance + " away");
    //System.out.println("The radius is " + radius);
    //System.out.println("The Power Port coordinates are: " + "X: " + powerPortY + " Y: " + powerPortY);
  }

  public double getConesCX() {
    return conesCX;
  }

  public double getConesCY() {
    return conesCY;
  }

  public double getCubesCX(){
    return cubesCX;
  }

  public double cubesCY(){
    return cubesCY;
  }

  public double getRedRobotCX() {
    return redRobotCX;
  }

  public double getRedRobotCY(){
    return redRobotCX;
  }

  public double getBlueRobotCX() {
    return blueRobotCX;
  }

  public double getBlueRobotCY() {
    return blueRobotCX;
  }

  public double getObjectDistance(){
    return object_distance;
  }

  public double getRobotDistance() {
    return robot_distance;
  }

  //1 is red, 2 is blue, 3 is shoot
 
  
  public void resetValues(){
    conesCX= 0; conesCY= 0; cubesCX= 0; cubesCY= 0; 
  redRobotCX= 0; redRobotCY= 0; blueRobotCX= 0; blueRobotCY= 0; object_distance= 0; robot_distance = 0;
  }

  public boolean checkCentered(String type){
    if(type.equalsIgnoreCase("CONE") ){
      if(getConesCX() >= 300 && getConesCX() <= 340){
        return true;
      }
      else if(((getConesCX() < 300) & (getConesCX() >= 0)) || ((getConesCX() > 340) & (getConesCX() <= 640))){
        return false;
      }
      return false;
  }
    else if (type.equalsIgnoreCase("CUBE") ){
      if(getCubesCX() >= 280 && getCubesCX() <= 320){
        System.out.print("CENTERED");
        return true;
      }
      
      else if(((getCubesCX() < 280) & (getCubesCX() > 320)) || ((getCubesCX() > 340) & (getCubesCX() <= 640))){
        System.out.print("NOT CENTERED");

        return false;
      }
      return false;
    }
    return false;
  }

/** 
 
  public void reportValues()
  {

      NetworkTableEntry visionShuffleboard;
    visionShuffleboard.setDouble(getBallXValue());
      ballYShuffleboard.setDouble(getBallYValue());
      ballDShuffleboard.setDouble(getBallDistance());
      ballRShuffleboard.setDouble(getBallRadius());
      targetXShuffleboard.setDouble(getTargetXValue());
      targetYShuffleboard.setDouble(getTargetYValue());
      targetDShuffleboard.setDouble(getTargetDistance());
      robotXShuffleboard.setDouble(getRobotXValue());
      robotYShuffleboard.setDouble(getRobotYValue());
      robotDShuffleboard.setDouble(getRobotDistance());
      shooterVModeShuffleboard.setNumber(getMode());
  }

  public void networkTablesShuffleboard() {
    //Network config
    
      ballXShuffleboard = networkTab.add("Ball CX", getBallXValue()).getEntry();  
      ballYShuffleboard = networkTab.add("Ball CY", getBallYValue()).getEntry();  
      ballDShuffleboard = networkTab.add("Ball Distance", getBallDistance()).getEntry();  
      ballRShuffleboard = networkTab.add("Ball Radius", getBallRadius()).getEntry();  
      targetXShuffleboard = networkTab.add("Target CX", getTargetXValue()).getEntry();  
      targetYShuffleboard = networkTab.add("Target CY", getTargetYValue()).getEntry();  
      targetDShuffleboard = networkTab.add("Target Distance", getTargetDistance()).getEntry(); 
      robotXShuffleboard = networkTab.add("Robot CX", getRobotXValue()).getEntry();  
      robotYShuffleboard = networkTab.add("Robot CY", getRobotYValue()).getEntry();  
      robotDShuffleboard = networkTab.add("Robot Distance", getRobotDistance()).getEntry();  
      shooterVModeShuffleboard = networkTab.add("Mode", getMode()).getEntry();
    
  }
  */
}