// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final int kDriverControllerPort = 0;
  }

  public static class ControllerConstants{
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double AXIS_THRESHOLD = 0.1;
    public static final double NEGITIVE_AXIS_THRESHOLD = -0.1;
  }

  public static class KinematicsConstants{

    // TODO: get these values too
    public static double TRACK_WIDTH = Units.inchesToMeters(21);
    public static double WHEEL_BASE = Units.inchesToMeters(25.5);
    
    public static SwerveDriveKinematics SWERVE_KINEMATIC = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE/ 2, TRACK_WIDTH / 2)
    );
  }

  public static class ModuleConstants{

    // these values are in meters, 
    /*
     * TODO: Remember to get these mesurements
     */
    public static final double WHEEL_DIAMTER_METERS = 0.037;
    public static final double DRIVE_GEAR_RATIO = 6.55;
    public static final double TURNING_GEAR_RATIO = 15.33;
    public static final double DRIVE_ENCODER_ROTATIONS = 0;
    public static final double TURNING_ENCODER_ROTATIONS = 0;
    public static final double DRIVE_ENCODER_SPEED = 0;
    public static final double TURNING_ENCODER_SPEED = 0;
    public static final double kP_TURNING_MOTOR = 0.1;
    public static final double kI_TURNING_MOTOR = 0.00;
    public static final double kD_TURNING_MOTOR = 0.000;



    public static final double MAX_SPEED = 6;
    public static final double MAX_TURNING_SPEED = 2 * 2 * Math.PI;
    public static final double MAX_TELEOP_SPEED = MAX_SPEED / 2;

    public static final double FRONT_LEFT_ENCODER_OFFSET = 0;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0;
    public static final double BACK_LEFT_ENCODER_OFFSET = 0;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0;
    

    public static final boolean FRONT_LEFT_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_ENCODER_REVERSED = false;

  }

  public static class PortConstants{
    public static final int FRONT_LEFT_THROTTLE = 7;
    public static final int FRONT_RIGHT_THROTTLE = 4;
    public static final int FRONT_LEFT_TURN = 2;
    public static final int FRONT_RIGHT_TURN = 8;
    public static final int BACK_LEFT_THROTTLE = 6;
    public static final int BACK_RIGHT_THROTTLE = 3;
    public static final int BACK_LEFT_TURN = 5;
    public static final int BACK_RIGHT_TURN = 1;
    
    public static final int FRONT_LEFT_ENCODER_PORT = 0;
    public static final int FRONT_RIGHT_ENCODER_PORT = 1;
    public static final int BACK_LEFT_ENCODER_PORT = 2;
    public static final int BACK_RIGHT_ENCODER_PORT = 3;

  }

  public static class PhysicalConstants{

  }

}
