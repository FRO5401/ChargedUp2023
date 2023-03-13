// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class lights extends SubsystemBase {
  /** Creates a new lights. */
    int length;
    AddressableLED led = new AddressableLED(0);

    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(15000);

    int rainbowFirstPixelHue = 223;

    
  
  public lights() {
  }

  public void solidColor(int h, int s, int v){
    length = 0;
    if(length == ledBuffer.getLength()){
    length =- length;
    }
    else{
    ledBuffer.setHSV(length, h, s, v ); 
    length =+ 1;   
    } 
  }


public void rainbow(){
  length = 0;

  if(length == ledBuffer.getLength()){
    length =- ledBuffer.getLength();
  }
  else{
    final int hue = (rainbowFirstPixelHue + (length * 180 / ledBuffer.getLength())) % 180;
    // Set the value
    ledBuffer.setHSV(length, hue, 255, 128);

    rainbowFirstPixelHue += 3;

    rainbowFirstPixelHue %= 180;
    length =+ 1;
    }
  }


  @Override
  public void periodic() {
 
  }
 
}

