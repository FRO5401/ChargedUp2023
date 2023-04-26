package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class rotateArmPID extends CommandBase {
    Arm arm;
    double leftPosition, rightPosition;
    boolean endCommand = false;

    public rotateArmPID(Arm m_arm, double m_leftPosition, double m_rightPosition){
        arm = m_arm;
        leftPosition = m_leftPosition;
        rightPosition = m_rightPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        if(arm.rotateLeftAtSetpoint(leftPosition) && arm.rotateLeftAtSetpoint(leftPosition)){
            endCommand = true;


        }
        else{
            arm.pidRotateArm(leftPosition, rightPosition);

        }
    }
    
    @Override
    public void end(boolean interrupted){


    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}