package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

public class armPosition extends Command {

  private final Arm arm;
  private boolean reached;
  private double position;
  
  public armPosition(Arm arm, double position) {
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
    if(arm.getPosition()>=position){
      arm.setSpeed(-.1);

      if(position-0.0002 <= arm.getPosition() && arm.getPosition() <= position+0.0002){
        reached = true;
      }
    }

    else if(arm.getPosition()<=position){
      arm.setSpeed(.1);

      if(position-0.0002 <= arm.getPosition() && arm.getPosition() <= position+0.0002){
        reached = true;
      } 
    }

    else{
      reached = true;
    } 
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

