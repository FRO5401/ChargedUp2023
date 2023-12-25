package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;


public class ZeroGround extends SequentialCommandGroup {
    Arm arm;

    public ZeroGround(Arm m_arm){
        this.arm = m_arm;
        addCommands(
            new ParallelRaceGroup(
                new WaitCommand(0.35),
                new translateArmPID(arm, 0)
            ),
            new ParallelRaceGroup(
                new WaitCommand(0.35),
                new rotateArmPID(arm, 0, 0)
            )
        );
    }
}