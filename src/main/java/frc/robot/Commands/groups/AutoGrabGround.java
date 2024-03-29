package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.LidarClaw;
import frc.robot.Commands.actions.translateArmPID;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class AutoGrabGround extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public AutoGrabGround(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;

        addCommands(

            new rotateArmPID(arm, 5, 5),
            new ParallelRaceGroup(
                new translateArmPID(m_arm, 15),
                new LidarClaw(claw, "")
            )
        );
    };

}