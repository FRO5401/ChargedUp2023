package frc.robot.Utilities;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Tabs {
    //Tabs
    public static ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");
    public static ShuffleboardTab testingTab = Shuffleboard.getTab("Testing");
    public static ShuffleboardTab commandTab = Shuffleboard.getTab("Commands");
    public static ShuffleboardTab graphTab = Shuffleboard.getTab("Graphs");
    public static ShuffleboardTab networkTab = Shuffleboard.getTab("Network Tables");

    //Entries
    //DRIVEBASE
    public static GenericEntry angleEntry, pitchEntry, yawEntry, rollEntry, rotationsEntry, turnRateEntry;
    public static GenericEntry leftSpeedEntry, rightSpeedEntry, speedEntry, leftPositionEntry, rightPositionEntry;
    public static GenericEntry shifterEntry, shifterComp;
    public static GenericEntry leftSpeedGraph, rightSpeedGraph, speedGraph, leftPositionGraph, rightPositionGraph, turnRateGraph;
    public static GenericEntry axisTester, pressureEntry;

    public static GenericEntry rotLeftSpeedEntry, rotRightSpeedEntry, transSpeedEntry, rotLeftPositionEntry, rotRightPositionEntry, transPositionEntry;

    //INFEED
    public static NetworkTableEntry gateEntry, gateComp;
    public static NetworkTableEntry infeedLeftSpeedEntry, infeedRightSpeedEntry, infeedLeftSpeedGraph, infeedRightSpeedGraph;

    //SHOOTER
    public static NetworkTableEntry shooterLeftSpeedEntry, shooterRightSpeedEntry, shooterLeftSpeedGraph, shooterRightSpeedGraph;
    public static NetworkTableEntry ballLoaderSpeedEntry, ballLoaderSpeedGraph;
    public static NetworkTableEntry shooterModeComp, shooterHighSpeedComp, shooterLowSpeedComp;

    //INTERNAL MECHs
    public static NetworkTableEntry internalMechAverageEntry, internalMechAverageGraph, leftIMSpeedGraph, rightIMSpeedGraph, leftIMSpeedEntry, rightIMSpeedEntry;

    //CLIMBER
    public static NetworkTableEntry transClimberLeftPositionEntry, transClimberRightPositionEntry, transClimberLeftPositionGraph, transClimberRightPositionGraph;
    public static NetworkTableEntry rotClimberLeftAngleEntry, rotClimberRightAngleEntry, rotClimberLeftRateEntry, rotClimberRightRateEntry;
    public static NetworkTableEntry rotClimberLeftAngleGraph, rotClimberRightAngleGraph, rotClimberLeftRateGraph, rotClimberRightRateGraph;

    //NETWORK TABLES
    public static NetworkTableEntry ballXShuffleboard, ballYShuffleboard, ballDShuffleboard, ballRShuffleboard;
    public static NetworkTableEntry targetXShuffleboard, targetYShuffleboard, targetDShuffleboard;
    public static NetworkTableEntry robotXShuffleboard, robotYShuffleboard, robotDShuffleboard;
    public static NetworkTableEntry shooterVModeShuffleboard;

    //ROBOT CONTAINER
    public static NetworkTableEntry chooserData;
}