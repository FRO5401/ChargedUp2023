package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

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


    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}