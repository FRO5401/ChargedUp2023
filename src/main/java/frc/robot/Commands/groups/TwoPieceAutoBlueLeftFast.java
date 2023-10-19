package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.AutoDriveFast;
import frc.robot.Commands.actions.AutoTurn;
import frc.robot.Commands.actions.LidarClaw;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;


public class TwoPieceAutoBlueLeftFast extends SequentialCommandGroup {
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

  public TwoPieceAutoBlueLeftFast(double DistanceInput, double SpeedInput, DriveBase passedDrivebase, Claw passedClaw, Arm passedArm) {
   drivebase = passedDrivebase;
    claw = passedClaw;
    arm = passedArm;

    addCommands(

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new translateArmPID(arm, 3.5)
      ),

      new AutoDriveFast(410, SpeedInput, drivebase),
      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new OffClaw(claw)

      ),
      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new rotateArmPID(arm, 5, 5)
      ),

      new ParallelRaceGroup(
        new WaitCommand(1),
        new translateArmPID(arm, -40)
      ),

      new ParallelCommandGroup(
        new AutoDriveFast(20, SpeedInput*0.3, passedDrivebase),
        new LidarClaw(passedClaw, null)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new translateArmPID(arm, 0)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new rotateArmPID(arm, 0, 0)
      ),

      new AutoDriveFast(420, -SpeedInput, drivebase),

      new AutoTurn(SpeedInput*0.8, 185, drivebase),
      new AutoDriveFast(30, SpeedInput*0.4, drivebase),

      new ParallelRaceGroup(
        new WaitCommand(0.5),
        new rotateArmPID(arm, 16, 16)
      ),

      new ParallelRaceGroup(
        new WaitCommand(0.75),
        new translateArmPID(arm, -50)

      ),

      new ParallelCommandGroup(
        new WaitCommand(0.1),
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