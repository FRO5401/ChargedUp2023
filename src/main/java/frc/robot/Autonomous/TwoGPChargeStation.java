package frc.robot.Autonomous;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.*;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Claw;
import frc.robot.Subsystems.DriveBase;

public class TwoGPChargeStation extends SequentialCommandGroup {

    private DriveBase drivebase;
    private Arm arm;
    private Claw claw;


    public TwoGPChargeStation(double DistanceInput, double speedInput, DriveBase passedDrivebase, Arm passedArm, Claw passedClaw, NetworkTable passedNetworkTable){
        addCommands(

        );
    }

}
