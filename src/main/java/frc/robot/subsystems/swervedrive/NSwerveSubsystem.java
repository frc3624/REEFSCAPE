// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.*;
import limelight.networktables.target.pipeline.FiducialTarget;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class NSwerveSubsystem extends SubsystemBase
{
  private final SwerveDrive swerveDrive;
  private boolean visionDriveTest = false;
  
  // Limelight components
  private Limelight limelight;
  private LimelightPoseEstimator poseEstimator;
  
  // Camera offset from robot center (adjust these values for your robot)
  private final Transform3d cameraOffset = new Transform3d(
      Units.inchesToMeters(0), // X offset
      Units.inchesToMeters(0), // Y offset  
      Units.inchesToMeters(0), // Z offset
      new Rotation3d(0, 0, 0)  // Rotation
  );

  public NSwerveSubsystem(File directory)
  {
    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16), Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, startingPose);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    
    // Initialize Limelight
    setupLimelight();
    
    if (visionDriveTest)
    {
      swerveDrive.stopOdometryThread();
    }
    
    setupPathPlanner();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
  }

  /**
   * Setup Limelight for AprilTag tracking and pose estimation
   */
  private void setupLimelight()
  {
    limelight = new Limelight("limelight");
    limelight.getSettings()
             .withLimelightLEDMode(LEDMode.PipelineControl)
             .withCameraOffset(cameraOffset)
             .save();
    poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
  }

  @Override
  public void periodic()
  {
    // Update odometry when vision is enabled
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
    }
    
    // Update robot orientation for MegaTag2
    limelight.getSettings()
             .withRobotOrientation(new Orientation3d(
                 new Rotation3d(0, 0, getHeading().getRadians()),
                 new AngularVelocity3d(
                     edu.wpi.first.units.Units.DegreesPerSecond.of(0),
                     edu.wpi.first.units.Units.DegreesPerSecond.of(0),
                     edu.wpi.first.units.Units.DegreesPerSecond.of(getRobotVelocity().omegaRadiansPerSecond)
                 )
             ))
             .save();
    
    // Get vision pose estimate and update odometry
    Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
    visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      // Only use vision if tags are close, multiple tags visible, and low ambiguity
      if (poseEstimate.avgTagDist < 4 && 
          poseEstimate.tagCount > 0 && 
          poseEstimate.getMinTagAmbiguity() < 0.3)
      {
        swerveDrive.addVisionMeasurement(
            poseEstimate.pose.toPose2d(),
            poseEstimate.timestampSeconds
        );
      }
    });
  }

  /**
   * Command to drive toward the closest detected AprilTag using Limelight
   * @return Command that drives toward AprilTag
   */
  public Command driveToAprilTag()
  {
    return run(() -> {
      Optional<LimelightResults> resultOpt = limelight.getLatestResults();
      
      if (resultOpt.isPresent() && resultOpt.get().targets_Fiducials.length > 0)
      {
        LimelightResults result = resultOpt.get();
        FiducialTarget bestTarget = result.targets_Fiducials[0];
        
        // Get horizontal angle to target
        double yawDegrees = bestTarget.tx;
        double yawRadians = Math.toRadians(yawDegrees);
        
        // Proportional control for rotation
        double rotationSpeed = -yawRadians * 2.0;
        
        // Move forward while turning toward target
        double forwardSpeed = 1.0; // meters per second
        
        // Only move forward if reasonably aligned (within 15 degrees)
        if (Math.abs(yawDegrees) < 15)
        {
          drive(new Translation2d(forwardSpeed, 0), rotationSpeed, false);
        }
        else
        {
          // Just rotate if not aligned
          drive(new Translation2d(0, 0), rotationSpeed, false);
        }
      }
      else
      {
        // No target found, stop
        drive(new Translation2d(0, 0), 0, false);
      }
    }).withName("DriveToAprilTag");
  }

  /**
   * Command to drive to a specific distance from an AprilTag using Limelight
   * @param targetDistanceMeters Desired distance from tag in meters
   * @return Command that approaches the AprilTag
   */
  public Command driveToAprilTagDistance(double targetDistanceMeters)
  {
    return run(() -> {
      Optional<LimelightResults> resultOpt = limelight.getLatestResults();
      
      if (resultOpt.isPresent() && resultOpt.get().targets_Fiducials.length > 0)
      {
        LimelightResults result = resultOpt.get();
        FiducialTarget bestTarget = result.targets_Fiducials[0];
        
        // Get target pose in robot space
        double currentDistance = bestTarget.targetPose_RobotSpace.getTranslation().getNorm();
        double yawDegrees = bestTarget.tx;
        double yawRadians = Math.toRadians(yawDegrees);
        
        // Distance error (positive = too far, negative = too close)
        double distanceError = currentDistance - targetDistanceMeters;
        
        // Proportional control for rotation
        double rotationSpeed = -yawRadians * 2.0;
        
        // Proportional control for forward speed
        double forwardSpeed = distanceError * 0.5;
        
        // Clamp speeds
        forwardSpeed = Math.max(-1.0, Math.min(1.0, forwardSpeed));
        rotationSpeed = Math.max(-2.0, Math.min(2.0, rotationSpeed));
        
        // Stop if within tolerance
        if (Math.abs(distanceError) < 0.1 && Math.abs(yawDegrees) < 5)
        {
          drive(new Translation2d(0, 0), 0, false);
        }
        else
        {
          drive(new Translation2d(forwardSpeed, 0), rotationSpeed, false);
        }
      }
      else
      {
        drive(new Translation2d(0, 0), 0, false);
      }
    }).withName("DriveToAprilTagDistance");
  }

  /**
   * Command to track a specific AprilTag ID
   * @param targetID The ID of the AprilTag to track
   * @param targetDistanceMeters Desired distance from tag
   * @return Command that tracks the specified AprilTag
   */
  public Command trackAprilTagByID(int targetID, double targetDistanceMeters)
  {
    return run(() -> {
      Optional<LimelightResults> resultOpt = limelight.getLatestResults();
      
      if (resultOpt.isPresent())
      {
        LimelightResults result = resultOpt.get();
        
        // Find the target with matching ID
        FiducialTarget targetTag = null;
        for (FiducialTarget tag : result.targets_Fiducials)
        {
          if (tag.fiducialId == targetID)
          {
            targetTag = tag;
            break;
          }
        }
        
        if (targetTag != null)
        {
          double currentDistance = targetTag.targetPose_RobotSpace.getTranslation().getNorm();
          double yawDegrees = targetTag.tx;
          double yawRadians = Math.toRadians(yawDegrees);
          double distanceError = currentDistance - targetDistanceMeters;
          
          double rotationSpeed = -yawRadians * 2.0;
          double forwardSpeed = distanceError * 0.5;
          
          forwardSpeed = Math.max(-1.0, Math.min(1.0, forwardSpeed));
          rotationSpeed = Math.max(-2.0, Math.min(2.0, rotationSpeed));
          
          if (Math.abs(distanceError) < 0.1 && Math.abs(yawDegrees) < 5)
          {
            drive(new Translation2d(0, 0), 0, false);
          }
          else
          {
            drive(new Translation2d(forwardSpeed, 0), rotationSpeed, false);
          }
        }
        else
        {
          // Target ID not found
          drive(new Translation2d(0, 0), 0, false);
        }
      }
      else
      {
        drive(new Translation2d(0, 0), 0, false);
      }
    }).withName("TrackAprilTag_" + targetID);
  }

  /**
   * Get the Limelight instance
   * @return Limelight object
   */
  public Limelight getLimelight()
  {
    return limelight;
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
    setupLimelight();
  }

  @Override
  public void simulationPeriodic()
  {
  }

  public void setupPathPlanner()
  {
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(speedsRobotRelative,
                              swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                              moduleFeedForwards.linearForces());
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      );
    } catch (Exception e)
    {
      e.printStackTrace();
    }
    
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose)
  {
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    
    return AutoBuilder.pathfindToPose(pose, constraints,
                                     edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                             DoubleSupplier angularRotationX)
  {
    return run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * 
                            swerveDrive.getMaximumChassisAngularVelocity(),
                        true, false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}
