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
import static frc.robot.Constants.Motors.*;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkClosedLoopController;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leader = new SparkMax(leftMotor, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(rightMotor, MotorType.kBrushless);

  private final SparkAbsoluteEncoder encoder = leader.getAbsoluteEncoder();
  private SparkClosedLoopController armPIDController;
  
  //To Be Tuned
  private final SlewRateLimiter limiter = new SlewRateLimiter(25);

  public Arm() {
    armPIDController = leader.getClosedLoopController();
    configureMotors();
  }

  public void configureMotors(){
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig.encoder.positionConversionFactor(360).velocityConversionFactor(360);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leaderConfig.closedLoop.pid(0, 0, 0);
    leaderConfig.closedLoopRampRate(2);
    followerConfig.closedLoopRampRate(2);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getPosition(){
    double pos = encoder.getPosition();
    pos = Math.round(pos * 1000.0) / 1000.0;
    return pos;
  }

  public void setPosition(double pos){
    armPIDController = leader.getClosedLoopController();
    armPIDController.setReference(pos, SparkMax.ControlType.kPosition);
  }

  public void setSpeed(double speed){
    follower.set(speed);
    leader.set(speed);
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
