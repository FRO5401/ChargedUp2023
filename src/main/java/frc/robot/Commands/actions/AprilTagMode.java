package frc.robot.Commands.actions;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.DriveBase;

public class AprilTagMode extends CommandBase {
    DriveBase drivebase;
    boolean endCommand = false;

    public AprilTagMode(DriveBase m_drivebase){
        drivebase = m_drivebase;
        addRequirements(drivebase);

    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        //shooter.runSmart("START")

        drivebase.switchVisionMode(2);
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

