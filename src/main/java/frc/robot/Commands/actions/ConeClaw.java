package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class ConeClaw extends CommandBase {
    Claw claw;
    boolean endCommand = false;

    public ConeClaw(Claw  m_claw){
        claw = m_claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){

        claw.toggleClaw("CONE");
       
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

