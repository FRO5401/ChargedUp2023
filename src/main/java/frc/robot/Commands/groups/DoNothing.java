package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Subsystems.DriveBase;

public class DoNothing extends SequentialCommandGroup {

  private DriveBase drivebase;

  public DoNothing(DriveBase passedDrivebase) {
    this.drivebase = passedDrivebase;
    addCommands(new AutoDrive(0, 0.0, drivebase));
  }

}