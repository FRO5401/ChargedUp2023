package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class StationPickup2 extends CommandBase {
    Arm arm;
    Claw claw;
    boolean endCommand = false;

    public StationPickup2(Arm m_arm, Claw  m_claw){
        arm = m_arm;
        claw = m_claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START")

        claw.toggleClaw("CONE");
        Timer.delay(0.25);
        arm.pidRotateArm(60, 21);
        Timer.delay(0.5);
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