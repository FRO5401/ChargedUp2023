package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.GyroBalance;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveBase;


public class DoNothing extends SequentialCommandGroup {

  private DriveBase drivebase;


  public DoNothing(DriveBase passedDrivebase) {
    drivebase = passedDrivebase;
    addCommands(new AutoDrive(0, 0.0, passedDrivebase));
  }



   
}