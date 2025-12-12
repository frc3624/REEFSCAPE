// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitchConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class TimedIntake extends Command {
    private final Intake intake;
    private double speed;
    private double time;
    private Timer timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimedIntake(Intake intake, double speed, double time) {
    this.time = time;
    this.intake = intake;
    this.speed = speed;
   addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(speed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0, false);
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() >= time){
      return true;
    
    }
    return false;
  }
}
