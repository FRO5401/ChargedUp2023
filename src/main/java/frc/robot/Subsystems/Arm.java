// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 

//commented for everybot testing
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  // a solenoid for breaking
  private Solenoid brakes;

  // two motors that control the arms movement 
  private CANSparkMax leftArm;
  private CANSparkMax rightArm;
  private CANSparkMax translationalArm;

  // a boolean to toggle for the breaks
  private boolean toggle;
  /** Creates a new arm. 
  public arm() {
    //init those objects from before
    brakes = new Solenoid(null, Constants.solenoidConstants.BREAK_SOLENOID);
    leftArm = new CANSparkMax(Constants.motorConstants.MOTER7, MotorType.kBrushless);
    rightArm = new CANSparkMax(Constants.motorConstants.MOTER8, MotorType.kBrushless);
    translationalArm = new CANSparkMax(Constants.motorConstants.MOTER9, MotorType.kBrushed);
    toggle = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  // takes in a string called mode that will determine how it runs, condensing what would be 3 methods to one.
  public void armMove(String mode){
    if(mode.equalsIgnoreCase("BREAK")){
      toggle = !toggle;
      brakes.set(toggle);
    }
    else if (mode.equalsIgnoreCase("FORWARD")){
      leftArm.set(-Constants.speedConstants.ARM_SPEED);
      rightArm.set(Constants.speedConstants.ARM_SPEED);
    }
    else if (mode.equalsIgnoreCase("REVERSE")){
      leftArm.set(Constants.speedConstants.ARM_SPEED);
      rightArm.set(-Constants.speedConstants.ARM_SPEED);

    }
    

    }
    public void translationalArmMove(String mode){
      if(mode.equalsIgnoreCase("Forward")){
        translationalArm.set(Constants.speedConstants.ARM_SPEED);
      }
      else if(mode.equalsIgnoreCase("Reverse")){
        translationalArm.set(-Constants.speedConstants.ARM_SPEED);
      }
  }
}
*/
