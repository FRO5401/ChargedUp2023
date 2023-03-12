package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static final String String = null;

    private Arm arm;
    private Solenoid firstStage;
    private Solenoid secondStage;
    private Lidar lidar;
    private I2C.Port i2cPort;


    public Claw(){
        firstStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        lidar = new Lidar(i2cPort);
        
    }

    public void toggleClaw(String mode){
        switch(mode){
            case "CUBE":
                firstStage.set(true);
                firstStage.set(false);

            break;
            case "CONE":
                firstStage.set(false); //FALSE CLOSES CLAW
                secondStage.set(false);
            break;
            case "OFF":
                firstStage.set(true); //TRUE MEANS OPEN CLAW
                secondStage.set(true);
            break;

        }
        
    }

    public void autoToggleClaw(String mode){
        if(lidar.getDistance() < 25){
        switch(mode){
            case "CUBE":
                firstStage.set(true);
                firstStage.set(false);

            break;
            case "CONE":
                firstStage.set(false); //FALSE CLOSES CLAW
                secondStage.set(false);
            break;
            case "OFF":
                firstStage.set(true); //TRUE MEANS OPEN CLAW
                secondStage.set(true);
            break;

        }
    }
    
        
    }



    public void reportLidarDistance(){
        System.out.println(lidar.getDistance());
    }




}
