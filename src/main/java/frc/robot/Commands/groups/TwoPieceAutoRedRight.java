package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoRedRight extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;


  public TwoPieceAutoRedRight(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    drivebase = passedDrivebase;

    addCommands(

      new AutoDrive(430,-SpeedInput, drivebase),

      new WaitCommand(0.5),

      new AutoDrive(410, SpeedInput, drivebase),

      new AutoDrive(10, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, -85, drivebase),

      new AutoDrive(270, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, 85, drivebase),

      new AutoDrive( -SpeedInput, 270, drivebase),

      new GyroBalance(drivebase, -SpeedInput*0.8)

    );
  }
}