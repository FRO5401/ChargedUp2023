package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AprilTagMode;
import frc.robot.Commands.AutoAlign;
import frc.robot.Commands.ConeClaw;
import frc.robot.Commands.ConeMode;
import frc.robot.Commands.CubeClaw;
import frc.robot.Commands.DriverMode;
import frc.robot.Commands.GroundPickup;
import frc.robot.Commands.GroundPickup2;
import frc.robot.Commands.LEDCommand;
import frc.robot.Commands.OffClaw;
import frc.robot.Commands.OperatorControl;
import frc.robot.Commands.StationPickup1;
import frc.robot.Commands.StationPickup2;
import frc.robot.Commands.XboxMove;
import frc.robot.Commands.ZeroGround;
import frc.robot.Commands.gearShiftHigh;
import frc.robot.Commands.gearShiftLow;
import frc.robot.Commands.rotateArmPID;
//import frc.robot.Commands.rotateArmUp;
import frc.robot.Commands.translateIn;
import frc.robot.Commands.translateOut;
import frc.robot.Commands.groups.CenterSinglePieceAuto;
import frc.robot.Commands.groups.DoNothing;
import frc.robot.Commands.groups.LowerNodePlace1;
import frc.robot.Commands.groups.LowerNodePlaceA;
//import frc.robot.Commands.groups.StationPickup2;

import frc.robot.Commands.groups.TwoPieceAutoBlueLeft;
import frc.robot.Commands.groups.TwoPieceAutoBlueRight;
import frc.robot.Commands.groups.TwoPieceAutoRedLeft;
import frc.robot.Commands.groups.TwoPieceAutoRedRight;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.*;
import frc.robot.Utilities.controllers.MultipleInputGroup;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

public class RobotContainer {
    
    private final SendableChooser<Command> chooser = new SendableChooser<Command>();
    // The robot's subsystems
    private final  DriveBase drivebase = new DriveBase();
    
    private final NetworkTables networktables = new NetworkTables();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();
    private final Lidar lidar;
    private final XboxMove xboxMove = new XboxMove(drivebase);

    //private final OperatorControl operator = new OperatorControl(arm, claw);

    //private final MultipleInputGroup drivetrain = new MultipleInputGroup();
    private final LEDCommand SolidLEDsCommand;
    private LEDSubsystem ledSubsystem;


    //private final CompressorSubsystem compressor = new CompressorSubsystem();


    public RobotContainer() {
        lidar = new Lidar(2);
        ledSubsystem = new LEDSubsystem(0);
        SolidLEDsCommand = new LEDCommand(ledSubsystem, LEDCommand.LEDPatterns.SolidLEDs);
        configureInputGroups();
        configureButtonBindings();
        drivebase.setDefaultCommand(xboxMove);

        
        chooser.setDefaultOption("AutoTwoPieceRedRight", new TwoPieceAutoRedRight(0, 0.8, drivebase));
        chooser.setDefaultOption("AutoTwoPieceRedLeft", new TwoPieceAutoRedLeft(0, 0.8, drivebase));
        chooser.setDefaultOption("CenterSinglePiece", new CenterSinglePieceAuto(0, 0.8, drivebase));
        chooser.setDefaultOption("AutoTwoPieceBlueRight", new TwoPieceAutoBlueRight(0, 0.8, drivebase));
        chooser.setDefaultOption("AutoTwoPieceBlueLeft", new TwoPieceAutoBlueLeft(0, 0.8, drivebase));
        chooser.setDefaultOption("DoNothing", new DoNothing());



        //chooser.setDefaultOption("DoNothing", new DoNothing(drivebase));
        //chooser.addOption("Trajectory Test", new SetTrajectoryPath(drivebase, "paths/DriveStraight.wpilib.json")); //REPLACE LATER
        SmartDashboard.putData("Auto choices", chooser);
    
        drivebase.resetEncoders();
        drivebase.resetGyroAngle();
    }

    public XboxMove getXboxMove(){
        return xboxMove;
    }
    public DriveBase getDriveBase(){
        return drivebase;
    }
    /* 
    public OperatorControl getOperator(){
        return operator;
    }
    */
    
    public Arm getArm(){
        return arm;
    }

    public Lidar getLidar(){
        return lidar;
    }

    public NetworkTables getNetworkTables(){
        return networktables;
    }
    
    public Claw getClaw(){
        return claw;
    }

    private void configureButtonBindings() {
        //Controls.driver.rightTrigger(0.05).onTrue(new XboxMove(drivebase));
        //Controls.driver.leftTrigger(0.05).onTrue(new XboxMove(drivebase));
        //Controls.driver.leftStick().onTrue(new XboxMove(drivebase));

       // CommandScheduler.getInstance().setDefaultCommand(ledSubsystem, SolidLEDsCommand);

        /*
        driver_Controller.a().whileTrue(new InstantCommand(()-> arm.resetArm()));
        driver_Controller.x().whileTrue(new InstantCommand(()->arm.ccwRotate()));
        driver_Controller.b().whileTrue(new InstantCommand(()->arm.cwRotate()));

        driver_Controller.leftBumper().whileTrue(new InstantCommand(()->arm.extendIn()));
        driver_Controller.rightBumper().whileTrue(new InstantCommand(()->arm.extendOut()));

        driver_Controller.pov(0).whileTrue(new InstantCommand(()->arm.stationPickup()));
        driver_Controller.pov(90).whileTrue(new InstantCommand(()->arm.groundPickup()));
        driver_Controller.pov(180).whileTrue(new InstantCommand(()->arm.lowerNodePlace()));
        driver_Controller.pov(270).whileTrue(new InstantCommand(()->arm.upperNodePlace()));
        */
        //Controls.operator.a().onTrue(Commands.parallel(new ZeroGround(arm, claw)));
        Controls.operator.povDown().whileTrue(Commands.parallel(new GroundPickup2(arm, claw)));//, new XboxMove(drivebase)));
        Controls.operator.povUp().onTrue(Commands.parallel(new ZeroGround(arm, claw)));
        Controls.operator.povLeft().onTrue(Commands.parallel(new LowerNodePlaceA(arm, claw)));
        Controls.operator.povRight().onTrue(Commands.parallel(new StationPickup2(arm, claw)));


        //Controls.operator.povLeft().whileTrue(Commands.parallel(new LowerNodePlaceA(arm, claw), new XboxMove(drivebase))).onFalse(Commands.parallel(new LowerNodePlaceB(arm, claw), new XboxMove(drivebase)));
        //Controls.operator.povLeft().whileTrue(new LowerNodePlace1(arm, claw).raceWith(new XboxMove(drivebase))).onFalse(new LowerNodePlace2(arm, claw));
        //Controls.operator.povDown().onTrue(new StationPickup1(arm, claw));
        //Controls.operator.povUp().whileTrue(Commands.parallel(new StationPickup1(arm, claw), new XboxMove(drivebase))).onFalse(Commands.parallel(new StationPickup2(arm, claw), new XboxMove(drivebase)));

        //Command repeats = (new XboxMove(drivebase)).repeatedly();

        //Claw mapping
        Controls.operator.y().onTrue(new OffClaw(claw));
        Controls.operator.rightTrigger().onTrue(new ConeClaw(claw));
        Controls.operator.leftTrigger().onTrue(new CubeClaw(claw));
        

        //Controls.driver.a().onTrue(new TwoPieceAutoRed(0, 0.8, drivebase));
        //Controls.driver.a().onTrue(new CenterSinglePieceAuto(0, 0.8, drivebase));
        //Controls.driver.a().onTrue(new TwoPieceAutoRedLeft(0, 0.8, drivebase));

        Controls.driver.povLeft().onTrue(new ConeMode(drivebase));
        Controls.driver.povRight().onTrue(new AprilTagMode(drivebase));
        Controls.driver.povDown().onTrue(new DriverMode(drivebase));
        Controls.driver.povUp().whileTrue(new AutoAlign(0.3, drivebase, networktables));


        // /Controls.driver.y().onTrue(new AutoAlign(left, drivebase, networkTables));

        //Controls.operator.rightBumper().onTrue(new rotateArmUp(arm));
        //Controls.operator.rightBumper().onTrue(new rotateArmPID(arm));

        //Controls.operator.x().onTrue(new translateIn(arm));
        //Controls.operator.b().onTrue(new translateOut(arm));


        //Controls.driver.back().onTrue(new gearShiftLow(drivebase));
        //Controls.driver.start().onTrue(new gearShiftHigh(drivebase));
    }

    private void configureInputGroups(){
    }
    /* 
    String trajectoryJSON = "New Path.wpilib.json";
    Trajectory trajectory = new Trajectory();
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
    */
    public Command getAutonomousCommand(){
        /* 
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
         }
          var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  Constants.DriveConstants.ksVolts,
                  Constants.DriveConstants.kvVoltSecondsPerMeter,
                  Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
              Constants.DriveConstants.kDriveKinematics,
              10);
      
              TrajectoryConfig config = new TrajectoryConfig(
                      Constants.DriveConstants.kMaxSpeedMetersPerSecond,
                      Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                  // Add kinematics to ensure max speed is actually obeyed
                  .setKinematics(Constants.DriveConstants.kDriveKinematics)
                  // Apply the voltage constraint
                  .addConstraint(autoVoltageConstraint);
                  RamseteCommand ramseteCommand =
                  new RamseteCommand(
                    examplePath,
                    drivebase::getPose,
                    new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        drivebase::getWheelSpeeds,
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    drivebase::drive,
                    drivebase);
                drivebase.resetGyroAngle();
      
                    // Run path following command, then stop at the end.
                    return ramseteCommand.andThen(() -> drivebase.drive(0, 0));
                    */
                    return null ;
    }

}