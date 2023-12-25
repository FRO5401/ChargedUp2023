package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoRedLeft extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;

  public TwoPieceAutoRedLeft(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    this.drivebase = passedDrivebase;
    addCommands(
      new AutoDrive(500,-SpeedInput, drivebase),

      new AutoDrive(450, SpeedInput, drivebase),

      new AutoDrive(45, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, 90, drivebase),

      new AutoDrive(270, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, -90, drivebase),

      new AutoDrive(480, -SpeedInput, drivebase),

      new GyroBalance(drivebase, -SpeedInput)
    );
  }
}