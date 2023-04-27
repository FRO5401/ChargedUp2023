// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */

/*
 Class used to create eaiser to read subsystems so it is not needed to keep remaking the objects
 */
public class SwerveModule {
    //the two motors in every swerve module 

    private final TalonFX driverMotor;
    private final TalonFX turningMotor;



    //PID controller used to control turning in auto, also used for setpoint 
    private final PIDController turningPIDControl;

    int absouluteEncoderID;
    boolean enable = true; 
    double currentLimitDrive = 30; 
    double triggerThresholdCurrentDrive = 10;
    double triggerThresholdTimeDrive = 1.5; 

    double currentLimitTurn = 10; 
    double triggerThresholdCurrentTurn = 5;
    double triggerThresholdTimeTurn = 1.5; 

    SupplyCurrentLimitConfiguration configDrive = new SupplyCurrentLimitConfiguration(enable, currentLimitDrive, triggerThresholdCurrentDrive, triggerThresholdTimeDrive);
    SupplyCurrentLimitConfiguration configTurn = new SupplyCurrentLimitConfiguration(enable, currentLimitTurn, triggerThresholdCurrentTurn, triggerThresholdTimeTurn);


   // private final AnalogInput absouluteEncoder;
     
   // private final double absouluteEncoderOffset;

    private boolean absouluteEncoderReversed;
    
    XboxController driver = RobotContainer.getDriverController();
    //Constuctor for init the modules
    public SwerveModule(int turningId, int driveId, boolean driveReversed, boolean turningReversed, int absouluteEncoderID, double absouluteEncoderOffset, boolean absouluteEncoderReversed){
        // this.absouluteEncoderOffset = absouluteEncoderOffset;
        this.absouluteEncoderReversed = absouluteEncoderReversed;
        
        // absouluteEncoder = new AnalogInput(absouluteEncoderID);

        driverMotor = new TalonFX(driveId);
        turningMotor = new TalonFX(turningId);

        driverMotor.configSupplyCurrentLimit(configDrive);
        driverMotor.configSupplyCurrentLimit(configTurn);

        driverMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);



        driverMotor.setInverted(driveReversed);
        turningMotor.setInverted(turningReversed);

        this.absouluteEncoderID = absouluteEncoderID;

        resetEncoders();
        
        
        turningPIDControl = new PIDController(ModuleConstants.kP_TURNING_MOTOR, ModuleConstants.kI_TURNING_MOTOR, ModuleConstants.kD_TURNING_MOTOR );
    
        turningPIDControl.enableContinuousInput(-Math.PI, Math.PI);

    }

    //methods for auto and path planning

    public double getDrivePosition(){
        return driverMotor.getSelectedSensorPosition() /2048;
    }

    public double getTurningPosition(){
        return turningMotor.getSelectedSensorPosition() /2048;
    }

    public double getDriveVelocity(){
        return driverMotor.getSelectedSensorVelocity() / (10/ 2048) * ModuleConstants.WHEEL_DIAMTER_METERS;
    }

    public double getTurningVelocity(){
        return turningMotor.getSelectedSensorVelocity();
    }

    
    public double getAbsoluteEncoderRad() {
        //using the average volatage draw of the encoder dividing it by the total voltage to get the position of thhe wheel based off the encoder
        double angle = Math.sin(getTurningPosition());
        angle *= 2 * Math.PI;
 
        return angle * (absouluteEncoderReversed ? -1.0 : 1);

    }
    

    public void resetEncoders() {
        driverMotor.setSelectedSensorPosition(0);

    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {

        if(Math.abs(state.speedMetersPerSecond) < 0.1){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driverMotor.set(ControlMode.PercentOutput,  state.speedMetersPerSecond * .5 );
        turningMotor.set(ControlMode.PercentOutput, turningPIDControl.calculate(getTurningPosition(), state.angle.getRadians()) );
        SmartDashboard.putString("Swerve " + absouluteEncoderID + " state", state.toString());
        SmartDashboard.putNumber(" calculation", turningPIDControl.calculate(getTurningPosition(), state.angle.getRadians()) );
        SmartDashboard.putNumber(" Encoder" + absouluteEncoderID, getTurningPosition());

    }

    public void stop(){
        driverMotor.set(ControlMode.PercentOutput,0);
        turningMotor.set(ControlMode.PercentOutput,0);
    }
}

// :)