package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.SparkMaxEncoderWrapper;
import main.java.edu.wpi.first.wpilibj.simulation.TiltedElevatorSim;
import main.java.edu.wpi.first.wpilibj.simulation.VariableLengthArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.*;
/**
 * This is a sample program to demonstrate how to use a state-space controller to control an arm.
 */
public class StateSpaceControlArm extends SubsystemBase {
    public StateSpaceControlArm() {
        initExtender();
        initPivot();
    }
    public void periodic() {
        
        pivotPeriodic();
        extendPeriodic();
    }
    private final Subsystem extendS = new Subsystem() {};
    private final CANSparkMax m_extendMotor = new CANSparkMax(Constants.ArmConstants.EXTEND_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_extendEncoderWrapper = new SparkMaxEncoderWrapper(m_extendMotor);
    private final LinearSystem<N2, N1, N1> m_extendPlant =
        LinearSystemId.identifyPositionSystem(
            12 / (Units.inchesToMeters(54.2)), 0.01);
    private final TiltedElevatorSim m_extendSim = new TiltedElevatorSim(
        m_extendPlant,
        DCMotor.getNEO(1),
        Constants.ArmConstants.EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION,
        Constants.ArmConstants.EXTEND_DRUM_RADIUS,
        Constants.ArmConstants.MIN_ARM_LENGTH, Constants.ArmConstants.MAX_ARM_LENGTH, true);
    
    private final LinearPlantInversionFeedforward<N2,N1,N1> m_extendFeedforward
        = new LinearPlantInversionFeedforward<>(m_extendPlant, 0.02);

    private final ProfiledPIDController m_extendController =
        new ProfiledPIDController(0.0955,0,0.12823,
            new Constraints(2, 4)
        );

    public void initExtender() {
        m_extendMotor.getEncoder().setPositionConversionFactor(Constants.ArmConstants.EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * Constants.ArmConstants.EXTEND_METERS_PER_DRUM_ROTATION);
        m_extendMotor.getEncoder().setVelocityConversionFactor(Constants.ArmConstants.EXTEND_DRUM_ROTATIONS_PER_MOTOR_ROTATION * Constants.ArmConstants.EXTEND_METERS_PER_DRUM_ROTATION / 60);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.MAX_ARM_LENGTH);
        m_extendMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.MIN_ARM_LENGTH);
        m_extendEncoderWrapper.setPosition(Constants.ArmConstants.MIN_ARM_LENGTH);
        m_extendController.reset(Constants.ArmConstants.MIN_ARM_LENGTH);
        m_extendSim.setState(VecBuilder.fill(Constants.ArmConstants.MIN_ARM_LENGTH, 0));
    }

    public void extendPeriodic(){}

    public void extendSimulationPeriodic() {
        m_extendSim.setAngleFromHorizontal(getAngle().getRadians());
        m_extendSim.setInputVoltage(m_extendMotor.getAppliedOutput());
        m_extendSim.update(0.02);
        m_extendEncoderWrapper.setSimPosition(m_extendSim.getPositionMeters());
        m_extendEncoderWrapper.setSimVelocity(m_extendSim.getVelocityMetersPerSecond());
    }

    public void setExtendVolts(double volts) {
        m_extendMotor.setVoltage(volts);
    }

    /**
     * Returns the distance from the pivot to the wrist joint in meters.
     * @return
     */
    public double getLengthMeters() {
        return m_extendEncoderWrapper.getPosition();
    }

    public double getExtendVelocity(){
        return m_extendEncoderWrapper.getVelocity();
    }

    public void setExtendVelocity(double velocityMetersPerSecond) {
        m_extendMotor.setVoltage(
            m_extendFeedforward.calculate(
                VecBuilder.fill(0, getExtendVelocity()),
                VecBuilder.fill(0, velocityMetersPerSecond)
            ).get(0,0)
            + Constants.ArmConstants.ARM_EXTEND_KG_VERTICAL * Math.sin(getAngle().getRadians())
        );
    }

    public void setExtendLength(double lengthMeters) {
        setExtendVelocity(
            m_extendController.calculate(getLengthMeters(), lengthMeters)
            +m_extendController.getSetpoint().velocity
        );
    }

    public Command extendC() {
        return extendS.run(()->{setExtendVolts(3);})
            .finallyDo((interrupted)->setExtendVolts(0));
    }

    public Command retractC() {
        return extendS.run(()->setExtendVolts(-3))
        .finallyDo((interrupted)->setExtendVolts(0));
    }

    // endregion

    // region pivot
    private final Subsystem pivotS = new Subsystem() {};

    private final CANSparkMax m_pivotMotor = new CANSparkMax(Constants.ArmConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxEncoderWrapper m_pivotEncoderWrapper = new SparkMaxEncoderWrapper(m_pivotMotor);

    private LinearSystem<N2, N1, N1> m_pivotPlant = LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(1),  1.0 / 3.0 * Constants.ArmConstants.ARM_MASS_KG * Constants.ArmConstants.MIN_ARM_LENGTH * Constants.ArmConstants.MIN_ARM_LENGTH
        , 1.0/Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION);

    private DCMotor m_pivotGearbox = DCMotor.getNEO(1);
    private final VariableLengthArmSim m_pivotSim = new VariableLengthArmSim(
        m_pivotPlant,
        DCMotor.getNEO(1),
        1.0/Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION,
        1.0 / 3.0 * Constants.ArmConstants.ARM_MASS_KG * Constants.ArmConstants.MIN_ARM_LENGTH * Constants.ArmConstants.MIN_ARM_LENGTH,
        Constants.ArmConstants.MIN_ARM_LENGTH,
        Constants.ArmConstants. MIN_ARM_ANGLE,
        Constants.ArmConstants.MAX_ARM_ANGLE,
        Constants.ArmConstants.ARM_MASS_KG,
        true
    );

    private LinearPlantInversionFeedforward<N2, N1, N1> m_pivotFeedForward
        = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);

    private ProfiledPIDController m_pivotController = new ProfiledPIDController(
        5, 0, 0, new Constraints(4,4));

    private double armStartAngle = 0;

    private void initPivot() {
        m_pivotMotor.getEncoder().setPositionConversionFactor(Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION);
        m_pivotMotor.getEncoder().setVelocityConversionFactor(Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION / 60);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.ArmConstants.MAX_ARM_ANGLE);
        m_pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.ArmConstants.MIN_ARM_ANGLE);

        m_pivotEncoderWrapper.setPosition(armStartAngle);
        m_pivotSim.setState(VecBuilder.fill(armStartAngle,0));

        m_pivotController.reset(armStartAngle);
        m_pivotController.setTolerance(0.05, 0.05);

        //pivotS.setDefaultCommand();
    }

    private void pivotPeriodic() {
        updatePivotPlant();
        m_pivotFeedForward = new LinearPlantInversionFeedforward<>(m_pivotPlant, 0.02);
    }
    private void pivotSimulationPeriodic() {
            m_pivotSim.setCGRadius(getLengthMeters() / 2);
            m_pivotSim.setMOI(getPivotMOI());
            m_pivotSim.setInputVoltage(DriverStation.isEnabled() ? m_pivotMotor.getAppliedOutput() : 0);
            m_pivotSim.update(0.02);
            m_pivotEncoderWrapper.setSimPosition(m_pivotSim.getAngleRads());
            m_pivotEncoderWrapper.setSimVelocity(m_pivotSim.getVelocityRadPerSec());
    }

    public void setPivotVolts(double volts) {
        m_pivotMotor.setVoltage(volts);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_pivotEncoderWrapper.getPosition());
    }

    public double getPivotVelocity() {
        return m_pivotEncoderWrapper.getVelocity();
    }

    public double getPivotMOI() {
        return 1.0 / 3.0 * Constants.ArmConstants.ARM_MASS_KG * getLengthMeters() * getLengthMeters();
    }

    public void updatePivotPlant() {
        m_pivotPlant.getA().set(1, 1, 
          -1.0/Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION * 1.0/Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION
          * m_pivotGearbox.KtNMPerAmp
          / (m_pivotGearbox.KvRadPerSecPerVolt * m_pivotGearbox.rOhms * getPivotMOI()));
        m_pivotPlant.getB().set(1, 0, 
          1.0/Constants.ArmConstants.ARM_ROTATIONS_PER_MOTOR_ROTATION * m_pivotGearbox.KtNMPerAmp / (m_pivotGearbox.rOhms * getPivotMOI()));
    }

    public void setPivotVelocity(double velocityRadPerSec) {
        setPivotVolts(m_pivotFeedForward.calculate(VecBuilder.fill(0, getPivotVelocity()), VecBuilder.fill(0, velocityRadPerSec)).get(0,0) + (getPivotkG() * getAngle().getCos()));
    }

    public void setPivotAngle(double targetAngle) {
        // We need to convert this to -90 to 270.
        // We don't want continuous input, but we need the rollover point to be outside our range of motion.
        targetAngle = MathUtil.angleModulus(targetAngle);
        // now in range -180 to 180
        if (targetAngle <= -Math.PI/2) {
            targetAngle += 2 * Math.PI;
        }
        SmartDashboard.putNumber("armRequestAngle", targetAngle);
        var outputVelocity = m_pivotController.calculate(
            getAngle().getRadians(),
            targetAngle
        );
        SmartDashboard.putNumber("armError", m_pivotController.getPositionError());
        SmartDashboard.putNumber("armRequestVel", outputVelocity + m_pivotController.getSetpoint().velocity);
        setPivotVelocity(outputVelocity + m_pivotController.getSetpoint().velocity);
    }

    public double getPivotkG() {
        double minkG = Constants.ArmConstants.ARM_PIVOT_KG_MIN_EXTEND;
        double maxkG = Constants.ArmConstants.ARM_PIVOT_KG_MAX_EXTEND;

        double result = minkG;
        double s = (getLengthMeters() - Constants.ArmConstants.MIN_ARM_LENGTH) / (Constants.ArmConstants.MAX_ARM_LENGTH - Constants.ArmConstants.MIN_ARM_LENGTH);
        result += s * (maxkG - minkG);
        return result;
    }

    public Command holdC() {
        return pivotS.run(()->setPivotAngle(0));
    }

    public Command counterClockwiseC() {
        return pivotS.run(()->setPivotVelocity(0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }

    public Command clockwiseC() {
        return pivotS.run(()->setPivotVelocity(-0.5))
        .finallyDo((interrupted)->setPivotVolts(0));
    }
}
  