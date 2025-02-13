package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class realPosition extends Command {

  private final Arm arm;
  private boolean reached;
  private double position;
  
  public realPosition(Arm arm, double position) {
    this.arm = arm;
    reached = false;
    this.position = position;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    arm.setPosition(.3); 
  }

  @Override
  public void end(boolean interrupted) {

    arm.setSpeed(0);
    reached = false;

  }

  @Override
  public boolean isFinished() {
    return reached;
  }
}

