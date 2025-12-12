// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static frc.robot.Constants.PWM.*;
import static frc.robot.Constants.LED.*;


public class Led extends SubsystemBase {
  private final Spark blinkIn = new Spark(0);
  
  public void decideColor(){
    if(greenLight){
      setColor(green);
      return;
    }
    if(redLight){
      setColor(red);
      return;
    }
    if(lavaLight){
      setColor(lava);
      return;
    }
    if(oceanLight){
      setColor(ocean);
      return;
    }
  }

  public Led() {

  }





  public void setColor(double color){
    blinkIn.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    decideColor();
    //blinkIn.set(m_color);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
