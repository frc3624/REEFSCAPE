package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;


public class LimitSwitchConfig extends SubsystemBase {

    DigitalInput switch1 = new DigitalInput(8);
    DigitalInput switch2 = new DigitalInput(9);

  
    public LimitSwitchConfig()
    {


    }
    public boolean isFinished()
    {

        return switch1.get() || switch2.get();

    }

}