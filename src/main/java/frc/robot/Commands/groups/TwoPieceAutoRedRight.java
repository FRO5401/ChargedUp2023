package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.AutoDrive;
import frc.robot.Commands.AutoTurn;
import frc.robot.Commands.GyroBalance;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoRedRight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
  private Arm arm;


  public TwoPieceAutoRedRight(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    drivebase = passedDrivebase;


    addCommands(
      new AutoDrive(1060,-SpeedInput, passedDrivebase),

      //new AutoDrive(193, SpeedInput, passedDrivebase), 
      new AutoDrive(1060, SpeedInput, passedDrivebase), 

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