package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

import frc.robot.Subsystems.DriveBase;
import frc.robot.Subsystems.Lidar;

public class LidarClaw extends CommandBase {
    Claw claw; 
    boolean endCommand = false;

    public LidarClaw(Claw m_claw){
        claw = m_claw;
        addRequirements(claw);

    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START")
        claw.autoToggleClaw();


        //arm.pidRotateArm(60, 3);
    }
    
    @Override
    public void end(boolean interrupted){
    }
    @Override
    public boolean isFinished() {
      return endCommand;
    }
}

