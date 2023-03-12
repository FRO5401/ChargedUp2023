package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class LowerNodePlaceB extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public LowerNodePlaceB(Arm arm, Claw claw){
        addCommands(
            new OffClaw(claw),
            new rotateArmPID(arm, 60, 19),
            new translateArmPID(arm, 0),
            new ConeClaw(claw),
            new rotateArmPID(arm, 0, 0)
        
        
        );
    }
}