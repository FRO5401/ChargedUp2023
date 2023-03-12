package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;


public class LowerNodePlaceA extends SequentialCommandGroup {
    Arm arm;
    Claw claw;

    public LowerNodePlaceA(Arm arm, Claw claw){
        addCommands(
            new rotateArmPID(arm, 18, 60),
            new translateArmPID(arm, -40)
            //Timer.delay(0.75);
        );
    };

}