package frc.robot.Commands.groups;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class LowerNodePlaceA extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public LowerNodePlaceA(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;

        addCommands(
            new rotateArmPID(arm, 18, 18)
            //Timer.delay(0.75);
        );
    };

}