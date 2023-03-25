package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

public class CubeClaw extends CommandBase {
    Claw claw;
    boolean endCommand = false;

    public CubeClaw(Claw  m_claw){
        claw = m_claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START")

        claw.toggleClaw("CUBE");
       
        //arm.pidRotateArm(60, 3);
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

