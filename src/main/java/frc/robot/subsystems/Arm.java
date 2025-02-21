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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import static frc.robot.Constants.Motors.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax leader = new SparkMax(leftMotor, MotorType.kBrushless);
  private final SparkMax follower = new SparkMax(rightMotor, MotorType.kBrushless);

  private final SparkAbsoluteEncoder encoder = leader.getAbsoluteEncoder();

  private ProfiledPIDController controller = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(.5, .5));
  private double lastSpeed = 0;
  private double lastTime = Timer.getFPGATimestamp();

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 192.71);

  public void goToPosition(double goalPosition){
    double pidVal = controller.calculate(encoder.getPosition(), goalPosition);
    double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
    leader.setVoltage(pidVal + feedforward.calculate(controller.getSetpoint().velocity, acceleration));
    lastSpeed = controller.getSetpoint().velocity;
    lastTime = Timer.getFPGATimestamp();
  }

  public void setSpeed(double speed){
    leader.set(speed);
  }

  public Arm() {
    configureMotors();
  }

  public void configureMotors(){
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leaderConfig.closedLoopRampRate(2);

    followerConfig.follow(leftMotor);

    follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Position", encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
