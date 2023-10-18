package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
public class Constants {


  public static class DriveConstants {
    //True Constants
    public static final double TELEOP_SPEED_ADJUSTMENT_RIGHT = 0.99;
    public static final double TELEOP_SPEED_ADJUSTMENT_LEFT = 1;
    public static final double LOW_GEAR_LEFT_DPP = 1; // Low gear skews left
    public static final double LOW_GEAR_RIGHT_DPP  = 1;
    public static final double HIGH_GEAR_LEFT_DPP = 1; // High gear skews left
    public static final double HIGH_GEAR_RIGHT_DPP = 1;
    public static final int DRIVEBASE_THRESHOLD_FOR_PID = 0;

    ////DRIVE MOTORS////
   //Right Motors
    public static final int DRIVE_MOTOR_RIGHT_1 = 1;
    public static final int DRIVE_MOTOR_RIGHT_2 = 3;
    public static final int DRIVE_MOTOR_RIGHT_3 = 5;

    //Left Motors
    public static final int DRIVE_MOTOR_LEFT_1 = 2;
    public static final int DRIVE_MOTOR_LEFT_2 = 4;
    public static final int DRIVE_MOTOR_LEFT_3 = 6;

    public static final int GEAR_SHIFTER = 0;

    public static final double kP = 1.199;
    public static final double kI = 0;
    public static final double KD = 0;



    public static final int ARM_MOTOR_LEFT = 8;
    public static final int ARM_MOTOR_RIGHT = 9;

    public static final int TRANS_MOTOR = 7;
    public static final double ksVolts = 0.10663;
    public static final double kvVoltSecondsPerMeter = 0.5;//2.2348;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16689;
    public static final double kTrackwidthMeters = 0.69;
    public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kPDriveVel = 1;
    public static final double kRamseteB = 1;
    public static final double kRamseteZeta = 0.7;

  }

  public class ControlConstants {
    //Operator Interfaces
    public static final double AXIS_THRESHOLD = 0.05;
    public static final int XBOX_CONTROLLER_DRIVER = 0;
    public static final int XBOX_CONTROLLER_OPERATOR = 1;

     // Buttons
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

    // Axes
    public static final int XBOX_AXIS_LEFT_X = 0;
    public static final int XBOX_AXIS_LEFT_Y = 1;
    public static final int XBOX_AXIS_LEFT_TRIGGER = 2;
    public static final int XBOX_AXIS_RIGHT_TRIGGER = 3;
    public static final int XBOX_AXIS_RIGHT_X = 4;
    public static final int XBOX_AXIS_RIGHT_Y = 5;

    //Drive Sensitivities
    public static final double DRIVE_SENSITIVITY_PRECISION = 0.5;
    public static final double DRIVE_SENSITIVITY_DEFAULT = 1;
    public static final double SPIN_SENSITIVITY = 0.7;

    //public static final int PCM_ID = 0;

    //Solenoids


    ////Sensors////
    //Encoders
    public static final int DRIVE_ENC_LEFT_A = 3;
    public static final int DRIVE_ENC_RIGHT_A = 1;
    public static final int DRIVE_ENC_LEFT_B = 4;
    public static final int DRIVE_ENC_RIGHT_B = 2;
  }
  public static class ArmConstants{
    public static final int EXTEND_MOTOR_ID = 10;

        //Arm length measured from shoulder pivot to wrist pivot
        public final static double MIN_ARM_LENGTH = Units.inchesToMeters(25);
        public final static double MAX_ARM_LENGTH = Units.inchesToMeters(54);

        public final Translation2d ARM_PIVOT_TRANSLATION = new Translation2d(0, Units.inchesToMeters(25));

        public final static double EXTEND_DRUM_RADIUS = Units.inchesToMeters(1);
        public static final double EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION = 1.0/14.667;
        public final static double EXTEND_METERS_PER_DRUM_ROTATION = Math.PI * 2 * EXTEND_DRUM_RADIUS;

        public static final double ARM_EXTEND_KG_VERTICAL = 0.2;
        /* PIVOT */
        public final static double MIN_ARM_ANGLE = Units.degreesToRadians(-45);
        public final static double MAX_ARM_ANGLE = Units.degreesToRadians(225);
        public static final double ARM_ROTATIONS_PER_MOTOR_ROTATION = 1.0/400.0;

        public static final double ARM_MASS_KG = 20;


        public static final int PIVOT_MOTOR_ID = 21;
        public final static double ARM_PIVOT_KG_MIN_EXTEND = 1.414 / 2 / Math.cos(Units.degreesToRadians(10.5));
        public final static double ARM_PIVOT_KG_MAX_EXTEND = 2.872 / 2 / Math.cos(Units.degreesToRadians(10.5));
  }
  public static class AutoConstants {
    //True Constants
    public static final double ANGLE_THRESHOLD = 6;
    public static final double AUTO_SPEED_ADJUSTMENT = 1.01;
    public static final double AUTO_TURN_SPEED = 0.8;
    public static final double AUTO_TURN_PRECISION = 0.5;
    public static final double ANGULAR_THRESHOLD = 2;

    //Trajectory Constants
    public static final double kTrackwidthMeters = 0.6858;
    public static final double ksVolts = 0.10663;
    public static final double kvVoltSecondsPerMeter = 2.2348;
    public static final double kaVoltSecondsSquaredPerMeter = 0.16689;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kPDriveVel = 1.05;
    public static final double SCUFFED_CORRECTION_CONSTANT = 0.26;
  }

  public static class SubsystemConstants {
    //Infeed
    public static boolean[] shuffleboardCompMode = {true, true, true, true, true, true};
    //public static boolean[] shuffleboardCompMode = {false, false, false, false, false, true};



    //Climber
    public static final int TRANS_MOTOR_1 = 13;
    public static final int TRANS_MOTOR_2 = 14;
    public static final int ROTATE_MOTOR_1 = 11;
    public static final int ROTATE_MOTOR_2 = 12;
    public static final int RATCHET_SOLENOID = 2;
    public static final int DIGITAL_INPUT_1 = 0;
    public static final int DIGITAL_INPUT_2 = 0;
    public static int ClimberStage = 0;


    //public static final double[]  TARGET_HEIGHT_METERS = [0.8382, 0.92, 1.17];
    //29.5in 60in
    //climber angle calculations

  }

}
