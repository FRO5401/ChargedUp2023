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
import frc.robot.Commands.actions.gearShiftHigh;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class SinglePieceAutoBlueLeftFast extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  private DriveBase drivebase;
  private Arm arm;
  private Claw claw;


  public SinglePieceAutoBlueLeftFast(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
    drivebase = passedDrivebase;
    claw = passedClaw;
    arm = passedArm;

    addCommands(

    new gearShiftHigh(drivebase),
    new ConeClaw(claw),
    new WaitCommand(0.25),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, 3.5)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new rotateArmPID(arm, 10, 10)
    ),

    new ParallelRaceGroup(
      new WaitCommand(0.5),
      new translateArmPID(arm, -5)
    ),
    new ParallelRaceGroup(
      new WaitCommand(0.25),
      new OffClaw(claw)

    ),
    new ParallelRaceGroup(
      new WaitCommand(0.25),
      new OffClaw(claw)

    ),
    new ParallelRaceGroup(
      new WaitCommand(0.25),
      new OffClaw(claw)

    ),

    new WaitCommand(0.5),

    new ParallelRaceGroup(
      new translateArmPID(arm, 0),
      new WaitCommand(0.35)

    ),

    new ParallelRaceGroup(
      new WaitCommand(0.25),
      new rotateArmPID(arm, 0, 0)
  ),
      new AutoDrive(500,-SpeedInput, passedDrivebase),

      new AutoDrive(450, SpeedInput, passedDrivebase),

      new AutoDrive(80, -SpeedInput, passedDrivebase),

      new AutoTurn(0.9*SpeedInput, 90, passedDrivebase),

      new AutoDrive(270, -SpeedInput, passedDrivebase),

      new AutoTurn(0.9*SpeedInput, -90, passedDrivebase),

      new AutoDrive( -SpeedInput, 490, passedDrivebase),

      new GyroBalance(passedDrivebase, -SpeedInput*0.8)
    );
  }
}