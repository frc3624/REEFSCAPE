package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class realPosition extends Command {

  private final Arm arm;
  private double position;
  private double speed;
  

  public realPosition(Arm arm, double position, double speed) {
    this.arm = arm;
    this.speed = speed;
    this.position = position;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setPosition(position);
  }

  @Override
  public void execute() {
    if(arm.getPosition() > position && arm.getPosition() < .9){
      arm.setSpeed(speed);
    }
    else if (arm.getPosition() < position || arm.getPosition() > .9){
      arm.setSpeed(-1*speed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    arm.goToPosition(0);
  }

  @Override
  public boolean isFinished() {
    return arm.reached();
  }
}

