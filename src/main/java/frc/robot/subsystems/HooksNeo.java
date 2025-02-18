// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Motors.downHook;

public class HooksNeo extends SubsystemBase {
  private SparkMax downHook = new SparkMax(downHook, MotorType.kBrushless);
  private SparkMax upHook = new SparkMax(upHook, MotorType.kBrushless);


  /** Creates a new HooksNeo. */
  public HooksNeo() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
