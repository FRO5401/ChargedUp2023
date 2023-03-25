package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class GroundPickup2 extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public GroundPickup2(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;
        addCommands(
        //shooter.runSmart("START");

        new OffClaw(claw),
        //new translateArmPID(m_arm, 0), 
        new ParallelRaceGroup(
            new WaitCommand(0.25),
            new rotateArmPID(m_arm, 4, 4)
        ),

        new ParallelRaceGroup(
            new WaitCommand(0.2),
            new translateArmPID(arm, -5)
        )
        //new translateArmPID(m_arm, 10)
        //new rotateArmPID(m_arm, 3, 3)





        );
    }
}