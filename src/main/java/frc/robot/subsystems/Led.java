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
  private final Spark blinkIn = new Spark(ledStrip);

  public Led() {
    confetti();
  }

  public void color(){
    if(coralLight){
        red();
        System.out.println("RED");
    }
    if(climbLight){
        green();
    }
    else{
        confetti();
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void red(){
    set(.59);
  }
  public void green(){
    set(.1);
  }
  public void confetti(){
    set(-0.87);
  }
  public void set(double speed){
    blinkIn.set(speed);
  }
  
}
