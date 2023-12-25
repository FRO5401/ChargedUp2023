package frc.robot.Commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.rotateArmPID;
import frc.robot.Subsystems.Arm;

public class StationPickup2 extends SequentialCommandGroup {
    Arm arm;

    public StationPickup2(Arm m_arm){
        arm = m_arm;
        addCommands(
            new rotateArmPID(arm, 13, 13)
        );
    }
}