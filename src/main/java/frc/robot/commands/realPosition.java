package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class realPosition extends Command {

  private final Arm arm;
  private double position;
  
  public realPosition(Arm arm, double position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setPosition(position);
  }

  @Override
  public void execute() {
    System.out.println("MOVINGI");
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    System.out.println("REACHD");
    return arm.reached();
  }
}

