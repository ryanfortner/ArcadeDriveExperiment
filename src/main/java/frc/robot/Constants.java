// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class DrivetrainConstants {
    public static final int LEFT_BACK_MOTOR_PORT = 1;
    public static final int LEFT_FRONT_MOTOR_PORT = 2;
    public static final int RIGHT_BACK_MOTOR_PORT = 3;
    public static final int RIGHT_FRONT_MOTOR_PORT = 4;
  }

  public static class AutoConstants {
    // Characterization values
    public static final double ksVolts = -0.045772;
    public static final double kvVoltSecondsPerMeter = 3.0212;
    public static final double kaVoltSecondsSquaredPerMeter = 0.42369;

    public static final double kPDriveVel = 3.839;

    public static final double kTrackwidthMeters = Units.inchesToMeters(23);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 0.1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
