package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class LowerNodePlace1 extends CommandBase {
    Arm arm;
    Claw claw;
    boolean endCommand = false;

    public LowerNodePlace1(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START");
        //claw.toggleClaw("CONE");
        arm.pidRotateArm(60, 18);
        //Timer.delay(0.75);
        arm.pidTranslateArm(-40);
        Timer.delay(1);
        
        /*
        Timer.delay(1);
        claw.toggleClaw("ON");
        arm.pidTranslateArm(0);
        Timer.delay(0.5);
        arm.pidRotateArm(0, 0);
        */

   
        //arm.pidRotateArm(60, 3);
        endCommand = false;
    }
    
    @Override
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}