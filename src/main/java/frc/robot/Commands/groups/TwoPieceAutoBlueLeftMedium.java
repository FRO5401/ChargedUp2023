package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoDriveFast;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.CubeClaw;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Commands.actions.LidarClaw;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.gearShiftHigh;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoBlueLeftMedium extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
  private Arm arm;
  private Claw claw;

  /**
   * @param DistanceInput
   * @param SpeedInput
   * @param passedDrivebase
   * @param passedClaw
   * @param passedArm
   */

  public TwoPieceAutoBlueLeftMedium(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
   drivebase = passedDrivebase;
    claw = passedClaw;
    arm = passedArm;

    addCommands(
     /* 
    new ConeClaw(claw), 
    new WaitCommand(0.25),
    */
    new gearShiftHigh(passedDrivebase),
    new ParallelRaceGroup(
      new BeginningTrans(passedArm),
      new AutoDrive(2, -SpeedInput*0.4, drivebase),
      new WaitCommand(0.2)
    ),
    

    
    /* 
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 10, 10)
    ),


    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, -5)

    ),
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new OffClaw(claw)

    ),
    */
    /* 
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, 0)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 0, 0)
    ),
    */
    //new AutoDrive(1000, SpeedInput, drivebase),
    new ParallelRaceGroup( 
      new OffClaw(claw),
      new WaitCommand(0.1)
    ),
    new AutoDriveFast(380, SpeedInput, drivebase),
    
  //new AutoTurn(SpeedInput, 172.5, drivebase),

    new ParallelRaceGroup(
      new WaitCommand(0.7),
      new rotateArmPID(arm, 4, 4)
    ),


    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, -48)
    ),

    new AutoDrive(35, SpeedInput*0.4, passedDrivebase),

    
   
    new ParallelRaceGroup(
      new WaitCommand(0.1),
      new ConeClaw(passedClaw)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.3),
      new translateArmPID(arm, 0)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.3),
      new rotateArmPID(arm, 0, 0)
    ),  

    new AutoDriveFast(390, -SpeedInput, drivebase),

    new AutoTurn(SpeedInput*0.55, 168, drivebase),//-163
    
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 17, 17)
    ),
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, -45)

    ),
    new AutoDrive(30, SpeedInput*0.6, drivebase),

    
    new ParallelCommandGroup(
      new WaitCommand(0.5),
      new OffClaw(claw)
    ),

    
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, 0)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 0, 0)
    )


    
    
    
   



   

  

    /*new ParallelRaceGroup(
      new CubeClaw(claw),
      new WaitCommand(0.5)
    ),

    new AutoDrive(-110, SpeedInput, drivebase),

    new ParallelRaceGroup(
      new OffClaw(claw),
      new WaitCommand(0.5)
    ),


    new ParallelRaceGroup(
      new AutoTurn(SpeedInput, 180, drivebase),
      new WaitCommand(0.5)
   ),
    

    new AutoDrive(1060, SpeedInput, drivebase) */


    
    );


    
    
    
  }


   
}