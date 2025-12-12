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
import com.revrobotics.AbsoluteEncoder;
import static frc.robot.Constants.Motors;
import static frc.robot.Constants.Motors.intakeFollower;
import static frc.robot.Constants.Motors.intakeLeader;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leader = new SparkMax(intakeLeader, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(intakeFollower, MotorType.kBrushless); 
  
  

  public Intake() {
    configureMotors();
  }

  public void configureMotors(){
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    
    leaderConfig.smartCurrentLimit(30);
    followerConfig.smartCurrentLimit(30);
    leaderConfig.idleMode(IdleMode.kBrake);
    followerConfig.idleMode(IdleMode.kBrake);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  public void set(double speed, boolean inverted){

    leader.set(speed);

    if(inverted){
      follower.set(speed*-1);
    }
    else{
      follower.set(speed);
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
}
