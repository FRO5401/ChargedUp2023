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


    public Claw(){
        firstStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        //lidar = new Lidar(source);
        
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
        if(lidar.getDistance() < 29){
            toggleClaw("CONE");
            return true;
        }
        else{
            return false;
        }
    }
    
       






}
