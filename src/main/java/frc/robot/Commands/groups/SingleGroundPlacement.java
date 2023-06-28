package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class SingleGroundPlacement extends SequentialCommandGroup {
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

  public SingleGroundPlacement(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
      drivebase = passedDrivebase;
      claw = passedClaw;
      arm = passedArm;

      addCommands(


      new ConeClaw(claw), 


      new WaitCommand(0.25),

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

      new WaitCommand(0.5),


    
      new ParallelRaceGroup(
        new translateArmPID(arm, 0),
        new WaitCommand(0.25)
        
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.25),
        new rotateArmPID(arm, 0, 0)
    ),

      
    new AutoDrive(480, -SpeedInput, passedDrivebase), 
    new GyroBalance(passedDrivebase, -SpeedInput*0.8)




    
    );






      /*
      For arm implementation:

      Parallel(
        new Auto(-162, SpeedInput, passedDrivebase), 
        new LowerNodePlaceA()
        ),
      new LowerNodePlaceB(),

      continue....

       */
      /* 
      new AutoDrive(12, SpeedInput, passedDrivebase), 
      new AutoTurn(SpeedInput, -90, passedDrivebase), 
      new AutoDrive(46, SpeedInput, passedDrivebase), 
      new AutoTurn(SpeedInput, 90, passedDrivebase), 
      new AutoDrive(100, SpeedInput, passedDrivebase)
      */


    
    
    
  }


   
}