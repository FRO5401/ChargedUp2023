package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class translateArmPID extends CommandBase {
    Arm arm;
    double transPos;
    boolean endCommand = false;

    public translateArmPID(Arm m_arm, double transPosition){
        arm = m_arm;
        transPos = transPosition;

        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        if((arm.transAtSetpoint(transPos))){
            endCommand = true;

        }
        else{
        arm.pidTranslateArm(transPos);

        }
    }
    
    @Override
    public void end(boolean interrupted){
        //arm.rotateArm(0, endCommand);

    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}