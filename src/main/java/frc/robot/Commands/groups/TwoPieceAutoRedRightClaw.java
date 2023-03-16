package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoTurn;
import frc.robot.Commands.CubeClaw;
import frc.robot.Commands.GyroBalance;
import frc.robot.Commands.LidarClaw;
import frc.robot.Commands.OffClaw;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.translateArmPID;
import frc.robot.Commands.translateOut;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoRedRightClaw extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
  private Arm arm;
  private Claw claw;


  public TwoPieceAutoRedRightClaw(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    drivebase = passedDrivebase;


    addCommands(
      new CubeClaw(claw),

      new ParallelCommandGroup(
        new rotateArmPID(arm, 19, SpeedInput),
        new translateArmPID(arm, 20)
      ),
      new OffClaw(claw),


      new translateArmPID(arm, 0),
      new rotateArmPID(arm, 0, SpeedInput),


      new AutoDrive(1000,-SpeedInput, passedDrivebase),

      new AutoTurn(SpeedInput, 180, passedDrivebase),
      new AutoGrabGround(arm, claw),
      new translateArmPID(arm, 10),
    
      new AutoTurn(SpeedInput, -180, passedDrivebase),



      //new AutoDrive(193, SpeedInput, passedDrivebase), 
      new AutoDrive(1060, SpeedInput, passedDrivebase), 
      new translateArmPID(arm, 20),
      new OffClaw(claw),

      new translateArmPID(arm, 0),
      new rotateArmPID(arm, 0, 0),


      new AutoDrive(80, -SpeedInput, passedDrivebase),

      new AutoTurn(SpeedInput, -85, passedDrivebase), 

      new AutoDrive(270, -SpeedInput, passedDrivebase), 

      new AutoTurn(SpeedInput, 84, passedDrivebase), 

      new AutoDrive(540, -SpeedInput, passedDrivebase), 

      new GyroBalance(passedDrivebase, -SpeedInput*0.8)






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




    );
    
    
    
  }


 


   
}