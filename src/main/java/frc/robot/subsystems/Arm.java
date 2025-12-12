// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import static frc.robot.Constants.Motors.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leader = new SparkMax(leftMotor, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(rightMotor, MotorType.kBrushless);

  public final SparkAbsoluteEncoder encoder;
  private double setpoint = 0;
  //private SparkClosedLoopController controller;

  private ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(1, 1));
  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();
  private ArmFeedforward feedforward = new ArmFeedforward(0.7, -0.27*Math.cos(50), 0.7);
  private ArmFeedforward feedforward2 = new ArmFeedforward(0.7, 0.15*Math.cos(50), 0.7);
  
  public void setVoltage(double volts){
    leader.setVoltage(-1*volts);
  }

  public void setPosition(double pos){
    this.setpoint = pos;
  }
  public double getPosition(){
    return encoder.getPosition();
  }
  public void goToPosition(double goalPosition){
    this.setpoint = goalPosition;
    double pidVal = controller.calculate(getPosition(), goalPosition);
    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    double feedforwardval = feedforward.calculate(controller.getSetpoint().velocity, acceleration);
    if(getPosition() >= 0.223 && getPosition() < 0.247){  
      feedforwardval = 0; 
    }
    else if(getPosition() > 0.245){
      feedforwardval = feedforward2.calculate(controller.getSetpoint().velocity, acceleration);
    }

    leader.setVoltage(pidVal + feedforwardval);
    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public void timedPosition(double time, double speed){
    Timer timer = new Timer();
    timer.start();

    setSpeed(speed);

    if(timer.get() >= 1){
      setSpeed(0);
      return;
    }
  }

  public void setPoint(double pos, double speed){
    setPosition(pos);
    if(getPosition() > pos){
      leader.set(-1*speed);
    }
    else{
      leader.set(speed);
    }

    if(reached()){
      return;
    }
  }

  public boolean reached(){
    if((setpoint - 0.003 < getPosition()) && (getPosition() < setpoint + 0.003)){
      return true;
    }
    return false;
  }

  public void setSpeed(double speed){
    leader.set(speed);
  }

  public void setPositionAuto(double pos){
    if(getPosition() > pos && !reached()){
      setSpeed(-0.25);
    }
    else if (getPosition() < pos && !reached()){
      setSpeed(0.25);
    }

  }

  public Arm() {
    configureMotors();
    //controller = leader.getClosedLoopController();
    encoder = leader.getAbsoluteEncoder();
  }

  
  public void configureMotors(){
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leaderConfig.closedLoop.pidf(0, 0, 0, 0);

    leaderConfig.closedLoopRampRate(4);

    followerConfig.follow(leftMotor);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /*public void setArmReference(double pos){
    controller.setReference(pos, ControlType.kPosition);
  }*/

  /*public void goToPosition(double pos){
    setArmReference(pos);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Position", getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}