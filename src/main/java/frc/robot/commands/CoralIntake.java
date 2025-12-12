// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitchConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.Led;
import static frc.robot.Constants.LED.*;


/** An example command that uses an example subsystem. */
public class CoralIntake extends Command {
    private final Intake intake;
    private double speed;
    private final XboxController xbox = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);
    private final Led led;
    private final LimitSwitchConfig limitSwitch;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralIntake(Intake intake, double speed, Led led, LimitSwitchConfig limitSwitch) {
    this.intake = intake;
    this.speed = speed;

    this.led = led;
    this.limitSwitch = limitSwitch;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(speed, false);
    //led.setColor(red);
    
    redLight = true;
    oceanLight = false;
    lavaLight = false;
    greenLight = false;
    led.decideColor();
    
    xbox2.setRumble(RumbleType.kBothRumble, 1);
    xbox.setRumble(RumbleType.kBothRumble, 1);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0, false);

    xbox.setRumble(RumbleType.kBothRumble, 0);
    xbox2.setRumble(RumbleType.kBothRumble, 0);

    redLight = false;

    redLight = false;
    if(highGear){
      oceanLight = true;
    }
    else{
      oceanLight = false;
      lavaLight = true;
    }

    led.decideColor();
    //while(limit.Clicked()){
      //led.setColor(green);
    //}

    //led.setColor(ocean);

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if(speed < 0 ){

      return false;

   }
   else{
    return (!(limitSwitch.isFinished()));
  }
}
}
