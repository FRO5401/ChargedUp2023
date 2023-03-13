// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public final class motorConstants{
  public static final int MOTER1 = 2;
  public final static int MOTER2 = 1;
  public final static int MOTER3 = 3;
  public final static int MOTER4 = 4;
   public final static int MOTER5 = 5;
  public final static int MOTER6 = 6;
  public final static int MOTER7 = 7;
  public static final int MOTER8 = 8;
  public static final int MOTER9 = 9;
  }

  public final class solenoidConstants{
  public static final int BREAK_SOLENOID = 0;
  public static final int LEFT_CLAW_SOLENOID = 1;
  public static final int RIGHT_CLAW_SOLENOID = 2;
  }
  public final class xboxConstants{
  public final static int DRIVER_PORT = 0;
  public final static int OPERATOR_PORT = 1;
  public static final double AXIS_THRESHOLD = 0.2;
  public static final int XBOX_CONTROLLER_DRIVER = 0;
  public static final int XBOX_CONTROLLER_OPERATOR = 1;
  public static final double PERCISION = 0.33;
  }
  public final class Buttons{
  public static final int XBOX_BUTTON_A = 1;
  public static final int XBOX_BUTTON_B = 2;
  public static final int XBOX_BUTTON_X = 3;
  public static final int XBOX_BUTTON_Y = 4;
  public static final int XBOX_BUTTON_LEFT_BUMPER = 5;
  public static final int XBOX_BUTTON_RIGHT_BUMPER = 6;
  public static final int XBOX_BUTTON_BACK = 7;
  public static final int XBOX_BUTTON_START = 8;
  public static final int XBOX_BUTTON_L3 = 9;
  public static final int XBOX_BUTTON_R3 = 10;
  }

  public final class Axes{
  public static final int XBOX_AXIS_LEFT_X = 0;
  public static final int XBOX_AXIS_LEFT_Y = 1;
  public static final int XBOX_AXIS_LEFT_TRIGGER = 2;
  public static final int XBOX_AXIS_RIGHT_TRIGGER = 3;
  public static final int XBOX_AXIS_RIGHT_X = 4;
  public static final int XBOX_AXIS_RIGHT_Y = 5;
  }
  public final class Sensitivity{
  public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
  public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
  public static final double SPIN_SENSITIVITY = 0.8;
  }
  public final class speedConstants{
  public static final double ARM_SPEED = 1;
  public static final double BALANCE_SPEED = 1; 
  }

  //Balancing constants
  public static final double ANGLE_THRESHOLD = 0;

  //Auto Constants
  public final static class pidConstants{
    public static final double ksVolts = 0.92477;
    public static final double kvVoltSecondsPerMeter = 2.6812;
    public static final double kaVoltSecondsSquaredPerMeter = 0.92615;
    public static final double kPDriveVel = 4.255;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kTrackwidthMeters = 0.69;
    public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  }
  
}
