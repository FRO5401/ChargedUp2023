package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ZeroGeneral extends CommandBase {
    Arm arm;
    boolean endCommand = false;

    public ZeroGeneral(Arm m_arm){
        arm = m_arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START");
        arm.pidTranslateArm(0);
        //claw.toggleClaw("CONE");
        Timer.delay(0.25);
        arm.pidTranslateArm(0);
        Timer.delay(1);
        arm.pidRotateArm(0, 0);
   
        //arm.pidRotateArm(60, 3);
        endCommand = true;
    }
    
    @Override
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}