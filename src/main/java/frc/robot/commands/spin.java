package frc.robot.commands;
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class spin extends Command {

  private final Arm arm;
  private double speed;
  private boolean finish;
  private double temp;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public spin(Arm arm, double speed) {
    this.arm = arm;
    this.speed = speed;
    temp = speed;

    finish = false;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if(arm.getPosition() >= 0.37){
      finish = true;
    }

    if(arm.getPosition() <= 0.1){
      finish = true;
    }*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if((arm.getPosition() >= 0.3 && arm.getPosition() <= .9 && speed < 0) || (arm.getPosition() > 0.9 && speed > 0)){
      temp = 0;
    }

    
    arm.setSpeed(temp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
    temp = speed;
    finish = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        return finish;
    }
}

