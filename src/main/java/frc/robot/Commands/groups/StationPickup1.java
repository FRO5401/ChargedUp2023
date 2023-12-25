package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Commands.actions.OffClaw;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class StationPickup1 extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public StationPickup1(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;

        addCommands(
            new OffClaw(claw),
            new rotateArmPID(arm, 25, 25)
        );
    };
}