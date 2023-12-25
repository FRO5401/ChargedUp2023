package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

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