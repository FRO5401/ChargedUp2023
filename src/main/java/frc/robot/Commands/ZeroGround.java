package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class ZeroGround extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public ZeroGround(Arm m_arm, Claw m_claw){
        addCommands(
        //shooter.runSmart("START");

        new OffClaw(claw),
        new translateArmPID(m_arm, 0), 
        new rotateArmPID(m_arm, 0, 0)
        );
    }
}