package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class translateIn extends CommandBase {
    Arm arm;
    boolean endCommand = false;

    public translateIn(Arm m_arm){
        arm = m_arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        arm.translateArm(-0.2, endCommand);
        
        endCommand = true;
    }
    
    @Override
    public void end(boolean interrupted){
        arm.translateArm(0, endCommand);

    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}