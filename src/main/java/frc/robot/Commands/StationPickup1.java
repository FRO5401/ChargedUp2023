package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class StationPickup1 extends SequentialCommandGroup {
    Arm arm;
    Claw claw;


    public StationPickup1(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;

        addCommands(
            new OffClaw(m_claw),
            new rotateArmPID(arm, 19, 19),
            new translateArmPID(arm, 20)
            //Timer.delay(0.75);
        );
    };
}