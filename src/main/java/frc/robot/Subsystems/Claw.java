// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//commented for everybot testing
/* 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class claw extends SubsystemBase {
  /** Creates a new claw. 

  //creating new solenoid device
  private Solenoid leftSolenoid;
  private Solenoid rightSolenoid;
  //boolean for toggling the solenoid
  private boolean toggle;


  public claw() {
    leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.solenoidConstants.LEFT_CLAW_SOLENOID);
    rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.solenoidConstants.LEFT_CLAW_SOLENOID);
    toggle = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void toggleClaw(String mode){
    if(mode.equalsIgnoreCase("CONE")){
      toggle = !toggle;
      leftSolenoid.set(toggle);
      rightSolenoid.set(toggle);
    }
    else if(mode.equalsIgnoreCase("CUBE")){
      toggle =!toggle;
      leftSolenoid.set(toggle);
    }
    else{
      toggle = false;
      leftSolenoid.set(toggle);
      rightSolenoid.set(toggle);
    }
  }
}
*/
