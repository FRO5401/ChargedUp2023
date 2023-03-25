package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.Commands.actions.translateOut;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class SinglePieceCenterMiddle extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
  private Arm arm;
  private Claw claw;


  public SinglePieceCenterMiddle(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
    drivebase = passedDrivebase;
    claw = passedClaw;
    arm = passedArm;

    addCommands(

    new ConeClaw(claw), 
    new WaitCommand(0.25),

    

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, 3.5)
    ),


    

    new ParallelRaceGroup(
      new WaitCommand(0.6),
      new rotateArmPID(arm, 15, 15)
    ),


    new ParallelRaceGroup(
      new WaitCommand(1),
      new translateArmPID(arm, -21)



    ),
    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new OffClaw(claw)

    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 15.5, 15.5)

    ),

    new WaitCommand(0.5),


  
    new ParallelRaceGroup(
      new translateArmPID(arm, 0),
      new WaitCommand(0.75)
      
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.25),
      new rotateArmPID(arm, 0, 0)
    ),

    
    new AutoDrive(650, -SpeedInput, passedDrivebase), 

    new AutoDrive(120, SpeedInput, passedDrivebase), 

    new ParallelRaceGroup(
      new GyroBalance(passedDrivebase, -SpeedInput*0.8),
      new WaitCommand(5)
    )
    
  );

  } 
}