// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.spin;
import frc.robot.commands.voltage;
import frc.robot.commands.realPosition;
import frc.robot.commands.TimedIntake;
import frc.robot.commands.liftStopper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimitSwitchConfig;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.HashMap;
import swervelib.SwerveInputStream;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import static frc.robot.Constants.LED.*;
import frc.robot.commands.speedToggle;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController driverXbox2 = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  
  //Subsystems
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  private final Arm arm = new Arm();
  private final Led led = new Led();  
  private final Intake Intake = new Intake();
  private final LimitSwitchConfig limit = new LimitSwitchConfig();

  //Commands
  //intake
  private final CoralIntake coralIntake = new CoralIntake(Intake, .4, led, limit);
  private final CoralIntake coralStop = new CoralIntake(Intake, 0, led, limit);
  private final CoralIntake coralDeposit = new CoralIntake(Intake, -.4, led, limit);
  private final AlgaeIntake algaeIntake = new AlgaeIntake(Intake, 0.4);
  private final AlgaeIntake algaeDeposit = new AlgaeIntake(Intake, -0.6);
  private final TimedIntake timedIntake = new TimedIntake(Intake, -.4, 2.25);
  private final TimedIntake parallelIntake = new TimedIntake(Intake, -.4, 4);
  private final realPosition positionAuto = new realPosition(arm, 0.11, 0.1);
  

  //arm positions
  
  //Add 0.007 if we change encoder stuff.
  
  //private final realPosition stationIntake = new realPosition(arm, 0.17);
  realPosition groundIntake = new realPosition(arm, 0, 0.325);

  private final realPosition depositL1 = new realPosition(arm, 0.1, 0.5);
  //private final realPosition dealgaefyL2 = new realPosition(arm, 0.275);

  private final realPosition L3 = new realPosition(arm, 0.325, 0.325);

  //DONT MAKE NEGATIVE, THE VALUE IS NEGATED IN COMMAND
  private final liftStopper stopper = new liftStopper(arm, 0.25, 0.25);
  private final liftStopper autoLift = new liftStopper(arm, 0.25, 0.875);
  private final liftStopper l3Lift = new liftStopper(arm, 0.25, 1);

  private final voltage idle = new voltage(arm, 0);
  
  //spin
  private final spin down = new spin(arm, .4);
  private final spin up = new spin(arm, -.4);
  
  //Climb
  


  /**R
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   private speedToggle lowSpeed = new speedToggle(driveAngularVelocity, 0.3, led, lava);
   private speedToggle highSpeed = new speedToggle(driveAngularVelocity, 1, led, ocean);

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("coralStop", coralStop);
    NamedCommands.registerCommand("coralIntake", coralIntake);
    NamedCommands.registerCommand("coralDeposit", coralDeposit);
    NamedCommands.registerCommand("algaeIntake", algaeIntake);
    NamedCommands.registerCommand("algaeDeposit", algaeDeposit);
    NamedCommands.registerCommand("depositL1", depositL1);
    NamedCommands.registerCommand("stationIntake", groundIntake);
    NamedCommands.registerCommand("timedIntake", timedIntake);
    NamedCommands.registerCommand("liftStopper", stopper);
    NamedCommands.registerCommand("autoStopper", autoLift);
    NamedCommands.registerCommand("L3", L3);
    NamedCommands.registerCommand("ff", idle);
    NamedCommands.registerCommand("positionAuto", positionAuto);
    NamedCommands.registerCommand("parallelIntake", parallelIntake);
    NamedCommands.registerCommand("l3Lift", l3Lift);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    arm.setDefaultCommand(idle);
    //Driver

    //Arm


    //Drive
    driverXbox.leftBumper().toggleOnTrue((Commands.runOnce(drivebase::zeroGyro)));

    //Operator

    driverXbox.povRight().onTrue(Commands.parallel(Commands.runOnce(() -> driveAngularVelocity.scaleTranslation(1)), Commands.runOnce(() -> oceanLight = true), Commands.runOnce(() -> lavaLight = false), Commands.runOnce(() -> highGear = true)));
    driverXbox.povUp().onTrue(Commands.parallel(Commands.runOnce(() -> driveAngularVelocity.scaleTranslation(0.3)), Commands.runOnce(() -> lavaLight = true), Commands.runOnce(() -> oceanLight = false), Commands.runOnce(() -> highGear = false)));

    //Arm
    driverXbox.rightTrigger().whileTrue(up);
    driverXbox.leftTrigger().whileTrue(down);
    
    driverXbox.a().whileTrue(drivebase.driveToAprilTag());

    driverXbox2.rightTrigger().whileTrue(up);
    driverXbox2.leftTrigger().whileTrue(down);

    driverXbox2.povDown().toggleOnTrue(groundIntake);
    driverXbox2.povRight().toggleOnTrue(depositL1);
    //driverXbox2.povUp().toggleOnTrue(dealgaefyL2);

    //Intake
    driverXbox2.a().toggleOnTrue(coralIntake);
    driverXbox2.x().toggleOnTrue(coralDeposit);
    driverXbox2.y().toggleOnTrue(algaeIntake);
    driverXbox2.b().toggleOnTrue(algaeDeposit);

    
    Command driveFieldOrientedAnglularVelocity    = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocitySim);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above! 
    }

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Copy of FrontGHAuto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}