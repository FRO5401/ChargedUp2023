package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private Solenoid firstStage;
    private Solenoid secondStage;
    private Lidar lidar;


    public Claw(Lidar m_lidar){
        firstStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
        lidar = m_lidar;
    }

    public void toggleClaw(String mode){
        switch(mode){
            case "CUBE":
                firstStage.set(true); //Activates one piston, creating a gap large enough for a cube to fit.
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
        if(firstStage.get() == true && secondStage.get() == true){ //If both cylinders are activated, the claw is deactivated.
            return true;
        }
        else{
            return false;
        }
    }

    // Toggles claw based off of distance
    public boolean autoToggleClaw(){
        double distance = lidar.getDistance();
        if(distance <= 43 && distance > 10){//38
            toggleClaw("CONE");
            return true;
        }
        else{
            return false;
        }
    }

}
