package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class ZeroGround extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public ZeroGround(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;
        addCommands(

        new ParallelRaceGroup(
            new WaitCommand(0.35),
            new translateArmPID(m_arm, 0)
        ),
        new ParallelRaceGroup(
            new WaitCommand(0.35),
            new rotateArmPID(m_arm, 0, 0)
        )
        
        );
        
    }
}