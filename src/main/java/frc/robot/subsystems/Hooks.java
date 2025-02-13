package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static frc.robot.Constants.PWM.downHookPort;
import static frc.robot.Constants.PWM.upHookPort;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;

import frc.robot.Constants.PWM.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hooks extends SubsystemBase{
    private final PWM upHook;
    private final PWM downHook;

    public Hooks(){
        downHook = new PWM(downHookPort);
        upHook = new PWM(upHookPort);
    }

    public void rotate(double angle, double angle1){
        downHook.setPosition(angle);
        upHook.setPosition(angle1);
    }
    public double getPosition(){
      return downHook.getPosition();

    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}
