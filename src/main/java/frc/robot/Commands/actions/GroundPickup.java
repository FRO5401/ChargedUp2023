package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;



public class GroundPickup extends CommandBase {
    Arm arm;
    Claw claw;
    boolean endCommand = false;

    public GroundPickup(Arm m_arm, Claw m_claw){
        arm = m_arm;
        claw = m_claw;
        addRequirements(arm, claw);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START");
        arm.pidRotateArm(60, 8);
        Timer.delay(1);
        arm.pidTranslateArm(-43);
        claw.toggleClaw("OFF");
        //claw.toggleClaw("ON");

   
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