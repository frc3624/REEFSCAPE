// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import static frc.robot.Constants.Motors.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leader = new SparkMax(leftMotor, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(rightMotor, MotorType.kBrushless);

  private final SparkAbsoluteEncoder encoder = leader.getAbsoluteEncoder();
  
  private double setpoint = 0;
  
  private SparkClosedLoopController armPIDController = leader.getClosedLoopController();
  
  public Arm() {
    configureMotors();
  }

  public void configureMotors(){
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    leaderConfig.closedLoop.pid(0.03, 0, 0);

    leaderConfig.closedLoopRampRate(2);
    followerConfig.closedLoopRampRate(2);

    followerConfig.follow(leftMotor);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPosition(){
    double position = encoder.getPosition();
    
    return position;
  }

  public void setPosition(double pos){
    this.setpoint = pos;


    armPIDController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setSpeed(double speed){
    follower.set(speed);
    leader.set(speed);
  }

  public boolean reached(){
    if((setpoint - .001 <= encoder.getPosition()) && (encoder.getPosition() <= setpoint + .001)){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Position", encoder.getPosition());
    SmartDashboard.putNumber("SetPosition", setpoint);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
