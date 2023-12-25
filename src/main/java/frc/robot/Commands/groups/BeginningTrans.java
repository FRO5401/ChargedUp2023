package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.actions.ZeroArm;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class BeginningTrans extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public BeginningTrans(Arm m_arm){
        arm = m_arm;
        addCommands(
            new translateArmPID(m_arm, 5),
            new ZeroArm(m_arm)
        );
    }
}