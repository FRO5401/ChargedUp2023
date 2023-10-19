package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.DriveBase;

public class PowerZero extends CommandBase {
    DriveBase drivebase;
    boolean endCommand = false;

    public PowerZero(DriveBase m_drivebase){
        drivebase = m_drivebase;
        addRequirements(drivebase);

    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){

        drivebase.gearShift("HIGH");

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

