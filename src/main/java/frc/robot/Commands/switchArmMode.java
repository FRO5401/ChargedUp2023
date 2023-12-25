package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

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