package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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


    //DigitalInput source = new DigitalInput(2);


    public Claw(Lidar m_lidar){
        firstStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        lidar = m_lidar;
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

    public boolean offMode(){
        if(firstStage.get() == true && secondStage.get() == true){
            return true;
        }
        else{
            return false;
        }
    }




    
    public boolean autoToggleClaw(){
        double distance = lidar.getDistance();
        if(distance <= 41 && distance > 10){
            toggleClaw("CONE");
            return true;
        }
        else{
            return false;
        }
    }
    
       






}
