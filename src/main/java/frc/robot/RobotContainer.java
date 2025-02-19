// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.Blink;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.spin;
import frc.robot.commands.Climb;
import frc.robot.commands.realPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hooks;
import frc.robot.subsystems.Limit;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

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
  private final Hooks hooks = new Hooks();
  private final Limit limit = new Limit();

  //Commands
  //intake
  private final CoralIntake coralIntake = new CoralIntake(Intake, .5, limit);
  private final CoralIntake coralDeposit = new CoralIntake(Intake, -.5, limit);
  private final AlgaeIntake algaeIntake = new AlgaeIntake(Intake, 0.5);
  private final AlgaeIntake algaeDeposit = new AlgaeIntake(Intake, -0.5);
  
  //arm positions
  private final realPosition groundIntake = new realPosition(arm, 0);
  private final realPosition stationIntake = new realPosition(arm, 0.3);
  private final realPosition depositL1 = new realPosition(arm, 0.2);
  private final realPosition dealgaefyL2 = new realPosition(arm, 0.006);
  //spin
  private final spin clockWise = new spin(arm, .4);
  private final spin counterClockWise = new spin(arm, -.4);
  
  //Climb
  private final Climb start = new Climb(hooks, 0.4, 0.6);
  private final Climb latch = new Climb(hooks, 0.9, 1);
  


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
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("CoralDeposit", coralDeposit);
    NamedCommands.registerCommand("CoralIntake", coralIntake);

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

    //Driver

    //Arm
    driverXbox.povUp().toggleOnTrue(dealgaefyL2);
    driverXbox.povRight().toggleOnTrue(depositL1);

    //Intake
    driverXbox.a().whileTrue(coralIntake);
    driverXbox.x().whileTrue(coralDeposit);

    //Drive
    driverXbox.leftBumper().toggleOnTrue((Commands.runOnce(drivebase::zeroGyro)));

    //Operator

    //Arm
    driverXbox.rightTrigger().whileTrue(clockWise);
    driverXbox.leftTrigger().whileTrue(counterClockWise);

    driverXbox2.rightTrigger().whileTrue(clockWise);
    driverXbox2.leftTrigger().whileTrue(counterClockWise);

    //Intake
    driverXbox2.a().toggleOnTrue(coralIntake);
    driverXbox2.x().toggleOnTrue(coralDeposit);
    driverXbox2.y().toggleOnTrue(algaeIntake);
    driverXbox2.b().toggleOnTrue(algaeDeposit);

    //Climb
    driverXbox2.leftBumper().toggleOnTrue(start);
    driverXbox2.rightBumper().toggleOnTrue(latch);

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
    return drivebase.getAutonomousCommand("Test Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
