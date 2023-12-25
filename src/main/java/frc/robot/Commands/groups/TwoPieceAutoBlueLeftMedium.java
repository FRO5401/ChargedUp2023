package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoDrive;
import frc.robot.Commands.actions.AutoDriveFast;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.gearShiftHigh;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoBlueLeftMedium extends SequentialCommandGroup {
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

  public TwoPieceAutoBlueLeftMedium(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
    this.drivebase = passedDrivebase;
    this.claw = passedClaw;
    this.arm = passedArm;

    addCommands(

      new gearShiftHigh(drivebase),
      new ParallelRaceGroup(
        new BeginningTrans(arm),
        new AutoDrive(2, -SpeedInput*0.4, drivebase),
        new WaitCommand(0.2)
      ),

      new ParallelRaceGroup(
        new OffClaw(claw),
        new WaitCommand(0.1)
      ),
      new AutoDriveFast(380, SpeedInput, drivebase),

      new ParallelRaceGroup(
        new WaitCommand(0.7),
        new rotateArmPID(arm, 4, 4)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new translateArmPID(arm, -48)
      ),

      new AutoDrive(35, SpeedInput*0.4, drivebase),

      new ParallelRaceGroup(
        new WaitCommand(0.1),
        new ConeClaw(claw)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.3),
        new translateArmPID(arm, 0)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.3),
        new rotateArmPID(arm, 0, 0)
      ),

      new AutoDriveFast(390, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput*0.55, 168, drivebase),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new rotateArmPID(arm, 17, 17)
      ),
      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new translateArmPID(arm, -45)

      ),
      new AutoDrive(30, SpeedInput*0.6, drivebase),

      new ParallelCommandGroup(
        new WaitCommand(0.5),
        new OffClaw(claw)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new translateArmPID(arm, 0)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new rotateArmPID(arm, 0, 0)
      )
    );
  }
}