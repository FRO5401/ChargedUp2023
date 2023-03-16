package frc.robot.Commands.groups;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class UpperNodePlace extends CommandBase {
    Arm arm;
    Claw claw;
    boolean endCommand = false;

    public UpperNodePlace(Arm m_arm, Claw m_claw){
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
        arm.pidRotateArm(60, 22);


   
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