package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

    public class StationPickup2 extends SequentialCommandGroup {
        Arm arm;
        Claw claw;
    
    
        public StationPickup2(Arm m_arm, Claw m_claw){
            arm = m_arm;
            claw = m_claw;
    
            addCommands(
                //new ConeClaw(m_claw),
                new rotateArmPID(arm, 13, 13)
                //new translateArmPID(arm, 0),
                //new rotateArmPID(m_arm, 0, 0)
                //Timer.delay(0.75);
            );
        }

    }
    