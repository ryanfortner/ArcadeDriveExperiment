// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Motor Controllers
  CANSparkMax leftFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.leftFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.leftBackMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.rightFrontMotorPort, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.rightBackMotorPort, CANSparkLowLevel.MotorType.kBrushless);

  // Encoders
  RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  // Combine groups into one drivetrain object
  public DifferentialDrive diffDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

  public Drivetrain() {
    // Factory-reset before each run, just in case
    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    // Set front/back motors on each side to follow each other. An alternative to motor control groups (deprecated).
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    // Each time code is deployed, set encoder values to 0.
    resetEncoders();

    // Inversions due to motor setup
    rightFrontMotor.setInverted(false);
    leftFrontMotor.setInverted(true);
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
    // This method will be called once per scheduler run
  }
  
  public void resetEncoders() {
    // sets position to 0.
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

}
