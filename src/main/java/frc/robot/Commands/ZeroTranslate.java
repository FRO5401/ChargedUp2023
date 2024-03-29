package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;



public class ZeroTranslate extends CommandBase {
    Arm arm;
    double leftPosition, rightPosition;
    boolean endCommand = false;

    public ZeroTranslate(Arm m_arm){
        arm = m_arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(arm.transAtSetpoint(0)){
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