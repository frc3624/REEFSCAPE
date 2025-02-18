// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.HooksNeo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbNeo extends Command {
  private final HooksNeo hooksNeo;
  private double upPose = 0;
  private double downPose = 0;

  public ClimbNeo(HooksNeo hooksNeo, double upPose, double downPose) {
    this.hooksNeo = hooksNeo;
    this.upPose = upPose;
    this.downPose = downPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hooksNeo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hooksNeo.hookPose(upPose, downPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hooksNeo.upReached() && hooksNeo.downReached();
  }
}
