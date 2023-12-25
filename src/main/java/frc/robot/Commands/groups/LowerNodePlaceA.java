package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.rotateArmPID;
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
        );
    };

}