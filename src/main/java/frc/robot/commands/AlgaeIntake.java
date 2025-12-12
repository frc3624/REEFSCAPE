package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeIntake extends Command {
  private final Intake intake;
  private double speed;

  public AlgaeIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.set(speed, true);
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
