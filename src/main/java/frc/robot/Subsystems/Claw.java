package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

    private static final String String = null;

    private Arm arm;
    private Solenoid firstStage;
    private Solenoid secondStage;

    public Claw(){
        firstStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
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







}
