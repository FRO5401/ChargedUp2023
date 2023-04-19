package frc.robot.Commands;

import javax.swing.GroupLayout.Group;

import org.ejml.dense.row.misc.TransposeAlgs_CDRM;

import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class rotateAndTranslate extends CommandBase {
    Arm arm;
    double leftPosition, rightPosition, transPosition;
    boolean endCommand = false;

    public rotateAndTranslate(Arm m_arm, double m_leftPosition, double m_rightPosition, double m_transPosition){
        arm = m_arm;
        leftPosition = m_leftPosition;
        rightPosition = m_rightPosition;
        transPosition = m_transPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        if(arm.rotateLeftAtSetpoint(leftPosition) && arm.rotateLeftAtSetpoint(leftPosition)  && arm.transAtSetpoint(transPosition)){
            endCommand = true;


        }
        else{
            arm.pidRotateArm(leftPosition, rightPosition);
            arm.pidTranslateArm(transPosition);
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