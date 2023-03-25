package frc.robot.Commands.groups;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.ConeClaw;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class LowerNode extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public LowerNode(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;
        addCommands(
            //new OffClaw(claw),
            /* 
            new ParallelRaceGroup(
                new WaitCommand(0.35),
                new rotateArmPID(arm, 16.5, 16.5)
            ),

            new ParallelRaceGroup(
                new WaitCommand(0.35),
                new translateArmPID(arm, -12)
            ), 
            */
            
            new rotateArmPID(arm, 19, 19)
            


           

            

        
        
        );
    }
}