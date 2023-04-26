package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class switchArmMode extends CommandBase {
    Arm arm;
    boolean endCommand = false;

    public switchArmMode(Arm m_arm){
        arm = m_arm;

        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        arm.setArmMode();
    }
    
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}