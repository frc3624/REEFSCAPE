// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;



import frc.robot.subsystems.Hooks;

/**
 * An example command that uses an example subsystem.
 */
public class Climb extends Command
{
    private final Hooks hooks;
    private double pos;
    private double pos2;

    public Climb(Hooks hooks, double pos, double pos2){
        this.hooks = hooks;
        this.pos = pos;
        this.pos2 = pos2;
    }

  @Override
  public void initialize()
  {
      //hooks.rotate(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    hooks.rotate(pos, pos2);
    System.out.println(hooks.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }


}

