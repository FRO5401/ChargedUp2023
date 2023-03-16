package frc.robot.Commands;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;

import frc.robot.Subsystems.DriveBase;

public class DriverMode extends CommandBase {
    DriveBase drivebase;
    boolean endCommand = false;

    public DriverMode(DriveBase m_drivebase){
        drivebase = m_drivebase;
        addRequirements(drivebase);

    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START")
        drivebase.activateDriverMode();
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

