// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.PWM.*;


public class Limit extends SubsystemBase {
  private final DigitalInput Limit = new DigitalInput(limitPort);
  private final DigitalInput Limit1 = new DigitalInput(limitPort1);

  public Limit() {
    
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean Clicked(){
    return Limit.get() || Limit1.get();
  }
}
