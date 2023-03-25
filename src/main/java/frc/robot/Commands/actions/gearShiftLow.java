package frc.robot.Commands.actions;

import javax.swing.GroupLayout.Group;
import java.lang.Thread;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;

public class gearShiftLow extends CommandBase {
    boolean endCommand = false;
    DriveBase drivebase;
    public gearShiftLow(DriveBase m_drivebase){
        drivebase = m_drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        //shooter.runSmart("START")

        drivebase.gearShift("LOW");
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

