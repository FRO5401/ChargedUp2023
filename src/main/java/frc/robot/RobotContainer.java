// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.Constants.pidConstants;
import frc.robot.Utilities.Controls;
import frc.robot.commands.Drivebase.Balance;
//import frc.robot.commands.Arm.armForward;
//import frc.robot.commands.claw.clawDrop;
// import frc.robot.commands.claw.conePickup;
//import frc.robot.commands.claw.cubePickup;
import frc.robot.subsystems.Drivebase;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.subsystems.arm;
// import frc.robot.subsystems.claw;
import edu.wpi.first.wpilibj2.command.Command;
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

  private static final Drivebase m_driveBase = new Drivebase();
  // private final claw Claw = new claw();
  // private final arm Arm = new arm();

  // Replace with CommandPS4Controller or CommandJoystick if needed


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    

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
    Controls.yButton.whileTrue(new Balance(m_driveBase));

    
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  String trajectoryJSON = "paths.Unnamed.wpilib.json";
  Trajectory trajectory = new Trajectory();
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            pidConstants.ksVolts,
            pidConstants.kvVoltSecondsPerMeter,
            pidConstants.kaVoltSecondsSquaredPerMeter),
        pidConstants.kDriveKinematics,
        10);

        TrajectoryConfig config = new TrajectoryConfig(
                pidConstants.kMaxSpeedMetersPerSecond,
                pidConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(pidConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            RamseteCommand ramseteCommand =
            new RamseteCommand(
              examplePath,
              m_driveBase::getPose,
              new RamseteController(pidConstants.kRamseteB, pidConstants.kRamseteZeta),
              new SimpleMotorFeedforward(
                  pidConstants.ksVolts,
                  pidConstants.kvVoltSecondsPerMeter,
                  pidConstants.kaVoltSecondsSquaredPerMeter),
                  pidConstants.kDriveKinematics,
                  m_driveBase::getWheelSpeeds,
              new PIDController(pidConstants.kPDriveVel, 0, 0),
              new PIDController(pidConstants.kPDriveVel, 0, 0),
              // RamseteCommand passes volts to the callback
              m_driveBase::tankDriveVolts,
              m_driveBase);
          m_driveBase.resetGyro();

              // Run path following command, then stop at the end.
              return ramseteCommand.andThen(() -> m_driveBase.tankDriveVolts(0, 0));
  }
  public Drivebase getDrivebase(){
    return m_driveBase;
  }
  /* 
   public Arm getArm(){
    return m_arm
  }
  */
}
