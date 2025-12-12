package frc.robot.commands;
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

/** An example command that uses an example subsystem. */
public class liftStopper extends Command {

  private final Arm arm;
  private double speed;
  private Timer timer;
  private double time;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public liftStopper(Arm arm, double speed, double time) {
    this.arm = arm;
    this.speed = speed;
    this.time = time;
    timer = new Timer();
    addRequirements(arm);
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

    arm.setSpeed(-speed);

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setSpeed(0);
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

