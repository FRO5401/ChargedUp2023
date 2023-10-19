package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Subsystems.DriveBase;


public class ReflectedTwoPieceAutoBlue extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;


  public ReflectedTwoPieceAutoBlue(double DistanceInput, double SpeedInput, DriveBase passedDrivebase) {
    this.drivebase = passedDrivebase;

    addCommands(
      new AutoDrive(1060,-SpeedInput, drivebase),

      new AutoDrive(1060, SpeedInput, drivebase),

      new AutoDrive(80, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, -85, drivebase),

      new AutoDrive(270, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput, 84, drivebase),

      new AutoDrive(540, -SpeedInput, drivebase),

      new GyroBalance(drivebase, -SpeedInput)

    );
  }
}