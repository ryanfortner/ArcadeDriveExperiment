// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrainSubsystem = new Drivetrain();
  private final DriveWithJoystick driveWithJoystick = new DriveWithJoystick(drivetrainSubsystem);

  public static Joystick joystick = new Joystick(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivetrainSubsystem.setDefaultCommand(driveWithJoystick);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to limit acceleration
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          AutoConstants.ksVolts, 
          AutoConstants.kvVoltSecondsPerMeter,
          AutoConstants.kaVoltSecondsSquaredPerMeter), 
          AutoConstants.kDriveKinematics, 
          6); // <- 6 volts, this can be changed, guide said 10

    // Create config for trajectory
    TrajectoryConfig config =
      new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // apply kinematics - ensures max speed is followed
        .setKinematics(AutoConstants.kDriveKinematics)
        // Apply voltage constraint
        .addConstraint(autoVoltageConstraint);
    
    // S-shaped trajectory (all in meters)
    Trajectory testTrajectory =
      TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing +x dir'n
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through two interior waypoints 
        List.of(new Translation2d(1, 0)), 
        // End 3 meters straight ahead, facing forward
        new Pose2d(1, 0, new Rotation2d(Math.PI * -2)), 
        // Pass config
        config);
    
    // This is the finicky part~ make sure to check all var names?
    RamseteCommand ramseteCommand = 
      new RamseteCommand(
        testTrajectory,
        drivetrainSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
          AutoConstants.ksVolts, 
          AutoConstants.kvVoltSecondsPerMeter, 
          AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        drivetrainSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback, so tankdrivevolts method required
        drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem);
    
    // Reset odometry to initial pose of trajectory,
    // run path, then stop at the end
    return Commands.runOnce(() -> drivetrainSubsystem.resetOdometry(testTrajectory.getInitialPose()))
      .andThen(ramseteCommand)
      .andThen(Commands.runOnce(() -> drivetrainSubsystem.tankDriveVolts(0, 0)));
  }
}
