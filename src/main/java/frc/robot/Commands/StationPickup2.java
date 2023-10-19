package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

    public class StationPickup2 extends SequentialCommandGroup {
        Arm arm;
        Claw claw;

        public StationPickup2(Arm m_arm){
            arm = m_arm;

            addCommands(
                new rotateArmPID(arm, 13, 13)
            );
        }
    }
