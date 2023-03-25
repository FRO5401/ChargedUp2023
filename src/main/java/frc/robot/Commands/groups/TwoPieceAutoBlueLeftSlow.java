package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.CubeClaw;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Commands.actions.LidarClaw;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoBlueLeftSlow extends SequentialCommandGroup {
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

  public TwoPieceAutoBlueLeftSlow(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
   drivebase = passedDrivebase;
    claw = passedClaw;
    arm = passedArm;

    addCommands(
     /* 
    new ConeClaw(claw), 
    new WaitCommand(0.25),
    */
    //new AutoDrive(10, -SpeedInput, drivebase),
    new ParallelRaceGroup(
      new WaitCommand(1),
      new translateArmPID(arm, 3.5)
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
    new AutoDrive(1000, SpeedInput, drivebase),
    new ParallelRaceGroup( 
      new OffClaw(claw),
      new WaitCommand(0.25)
    ),
  //new AutoTurn(SpeedInput, 172.5, drivebase),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 5, 5)
    ),


    new ParallelRaceGroup(
      new WaitCommand(1),
      new translateArmPID(arm, -40)
    ),

    new AutoDrive(100, SpeedInput*0.5, passedDrivebase),

    
    new WaitCommand(1),
    new CubeClaw(passedClaw),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, 0)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 0, 0)
    ),  

    new AutoDrive(1000, -SpeedInput, drivebase),

    new AutoTurn(SpeedInput*0.8, 173, drivebase),
    new AutoDrive(100, SpeedInput, drivebase),



    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 16, 16)
    ),


    new ParallelRaceGroup(
      new WaitCommand(0.75),
      new translateArmPID(arm, -40)

    ),


    
    new ParallelCommandGroup(
      new WaitCommand(1),
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