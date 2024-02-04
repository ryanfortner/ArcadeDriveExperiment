// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  // Motor Controllers
  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.LEFT_FRONT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.LEFT_BACK_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.RIGHT_FRONT_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.RIGHT_BACK_MOTOR_PORT, CANSparkLowLevel.MotorType.kBrushless);

  // Encoders
  private final Encoder m_leftEncoder = 
    new Encoder(
      DrivetrainConstants.LEFT_BACK_MOTOR_PORT,
      DrivetrainConstants.LEFT_FRONT_MOTOR_PORT);
  private final Encoder m_rightEncoder = 
    new Encoder(
      DrivetrainConstants.RIGHT_BACK_MOTOR_PORT,
      DrivetrainConstants.RIGHT_FRONT_MOTOR_PORT);

  // Combine groups into one drivetrain object
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public Drivetrain() {
    // Factory-reset before each run, just in case
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    // Inversions due to motor setup
    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);

    // Set front/back motors on each side to follow each other. An alternative to motor control groups (deprecated).
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    // Each time code is deployed, set encoder values to 0.
    resetEncoders();
    m_odometry =
      new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
  
  // resets drive encoders to 0
  public void resetEncoders() {
    // sets position to 0.
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // Returns the currently-estimated pose of robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Returns the current wheel speeds of the robot
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  // Resets odometry to specified pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  // Drive robot using arcade controls
  public void arcadeDrive(double fwd, double rot) {
    diffDrive.arcadeDrive(fwd, rot);
  }

  // Controls the left & right sides of the drive directly with voltage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // left volts
    leftBackMotor.setVoltage(leftVolts);
    leftFrontMotor.setVoltage(leftVolts);

    // right volts
    rightBackMotor.setVoltage(rightVolts);
    rightFrontMotor.setVoltage(rightVolts);

    diffDrive.feed();
  }

  // Gets average distance of two encoders
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  
  // Gets left drive encoder
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  // Gets right drive encoder
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  // Sets max output of drive
  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
  }

  // Zeros the heading of the robot
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Returns heading of the robot, in degrees from 180 - -180
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  // Returns turn rate of robot
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
