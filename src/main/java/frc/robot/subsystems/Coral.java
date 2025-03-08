package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.CanIds;
import frc.robot.commands.*;
import frc.utils.CommonLogic;
import frc.utils.RobotMath;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Member;
import java.util.EventListener;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import frc.robot.subsystems.ElevatorAndArm;
import frc.robot.subsystems.ElevatorAndArm.ElevAndArmPos;

public class Coral extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax coralMotor = new SparkMax(CanIds.CORAL_MOTOR, MotorType.kBrushless);

    public DigitalInput Coralhopper;
    public DigitalInput CoralIntake;

    private final double coralPosTol = 0.2;
    private double CoralCurPos = 0.0;
    private double CoralCmdPos = 0; // ElevAndArmPos.START.coralPos;
    // The public values are used to calculate the m and b values for coral holding
    //
    public final double CORAL_PICKUP_POS = -.35;
    public final double CORAL_LEVLE4_POS = 1.7;
    // The private values are used to final position the coral so the wheel is on
    // the mid-line of the coral which we currently believe is between the sensors
    //
    private final double UpperSensorIndexPos = CORAL_PICKUP_POS;
    private final double LowerSensorIndexPos = CORAL_PICKUP_POS;

    private double coral_m = 0;
    private double coral_b = 0;

    private final double SPEED_PRECORAL = 0.40;
    private final double SPEED_INDEXING = 0.25;
    private final double SPEED_ALGE_EXTRACT = 0.5;
    private final double SPEED_LEVEL1_DEPLOY = 0.7;
    private final double SPEED_LEVEL2_DEPLOY = 0.5;
    private final double SPEED_LEVEL3_DEPLOY = -0.5;
    private final double SPEED_LEVEL4_DEPLOY = -0.7;

    private final ClosedLoopSlot CORAL_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot CORAL_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot CoralCurrentSlot = CORAL_CLOSED_LOOP_SLOT_UP;

    private boolean isBrake = true;
    private CoralPhase currCoralPhase = CoralPhase.INIT;
    private ElevatorAndArm m_ElevatorAndArm = null;

    private double SensorTime = 0.0;

    public boolean isIntaking = false;

    public Coral() {
        configCoralMotor();
        Coralhopper = new DigitalInput(0);
        CoralIntake = new DigitalInput(1);
    }

    public void setElevatorAndArmSystem(ElevatorAndArm newElevatorAndArm) {
        m_ElevatorAndArm = newElevatorAndArm;
    }

    private void SetCoralIndex(double newPos) {
        coralMotor.set(0.0);
        coralMotor.getEncoder().setPosition(0);
        setCoralCmdPos(newPos);
    }

    // set y = mx + b for algae holding
    public void setM(double newM) {
        coral_m = newM;
    }

    // set y = mx + b for algae holding
    public void setB(double newB) {
        coral_b = newB;
    }

    private boolean isCoralMotorInPosiiton() {

        return (CommonLogic.isInRange(CoralCurPos, CoralCmdPos, coralPosTol));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        CoralCurPos = coralMotor.getEncoder().getPosition();

        switch (currCoralPhase) {

            case INIT:
                try {
                    if (/*getUpperSensor() &&*/ m_ElevatorAndArm.isElevatorAndArmAtTarget(ElevAndArmPos.PICKUP)) {
                        currCoralPhase = CoralPhase.HOLDING;
                    }
                } catch (NullPointerException e) {
                    System.err.println("Caught a NullPointerException: " + e.getMessage());
                    // Handle the exception, maybe log it or take alternative action
                    System.err.println("Coral Phase is in INIT and catching null pointer");
                }

                break;

            case PRECORAL:
                // Looking for the upper sensor to trip
                if (getUpperSensor()) {
                    // Sensor Tripped
                if (RobotMath.getTime() >= SensorTime) {
                    currCoralPhase = CoralPhase.CORAL_INDEX_WAITING;
                    coralMotor.set(SPEED_INDEXING);
                    m_ElevatorAndArm.setBlockMoves(true);
                } else {
                    // Sensor Tripped
                    SensorTime = RobotMath.getTime() + 0.0;
                   
                }
            }
                break;

            case CORAL_INDEX_WAITING:
                // Coral Index is complete when 1 of 2 things happens
                // We lost the upper sensor.
                if (!getUpperSensor()) {
                    // currCoralPhase = CoralPhase.CORAL_INDEX_COMPLETE;
                    // Coral has indexed off the upper sensor
                    SetCoralIndex(UpperSensorIndexPos);
                    currCoralPhase = CoralPhase.FINAL_POSITIONING;
                }

                // Lower sensor is less reliable because the wiper can trip it

                //if (getLowerSensor()) {
                    // set power 0 and index to the perfect coral hold position
                //    SetCoralIndex(LowerSensorIndexPos);
                //    currCoralPhase = CoralPhase.FINAL_POSITIONING;
                //}
                break;

            case FINAL_POSITIONING:
                if (isCoralMotorInPosiiton() && getLowerSensor()) {
                    currCoralPhase = CoralPhase.HOLDING;
                    m_ElevatorAndArm.setBlockMoves(false);
                    m_ElevatorAndArm.setNewPos(ElevAndArmPos.SAFETYPOS);
                }
                break;
            case HOLDING:
                setCoralCmdPos(calcCoralCompensation(m_ElevatorAndArm.getArmCurPos()));
                break;

            default:
                // do nothing... we are doing something that uses power set
                break;

        }

    }

    public void enabledInit() {
        coralMotor.getClosedLoopController().setReference(coralMotor.getEncoder().getPosition(), ControlType.kPosition);
        coralMotor.set(0);
    }

    public void SetElevatorAndArm(ElevatorAndArm sysElevatorAndArm) {
        m_ElevatorAndArm = sysElevatorAndArm;
        currCoralPhase = CoralPhase.HOLDING;
    }

    public String getCoralPhaseString() {
        return currCoralPhase.toString();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public boolean getIsHolding() {
        return currCoralPhase == CoralPhase.HOLDING;
    }

    public double getCoralCmdPos() {
        return (CoralCmdPos);
    }

    public void stopCmd() {

        setCoralCmdPos(CoralCurPos);

    }

    public void coastCmd() {

        if (isBrake) {

            // make all moters coast else make them brake.

        }

    }

    public double getCoralCurPos() {
        return (CoralCurPos);
    }

    private void setCoralCmdPos(double newPos) {
        this.CoralCmdPos = newPos;
        if (newPos > CoralCurPos) {
            CoralCurrentSlot = CORAL_CLOSED_LOOP_SLOT_UP;
        } else {
            CoralCurrentSlot = CORAL_CLOSED_LOOP_SLOT_DOWN;
        }
        coralMotor.getClosedLoopController().setReference(newPos,
                ControlType.kPosition, CoralCurrentSlot);
    }

    public double coralMotorPower() {
        return coralMotor.getOutputCurrent();
    }

    public void autonInit() {
        coralMotor.getEncoder().setPosition(CORAL_PICKUP_POS);
        currCoralPhase = CoralPhase.HOLDING;
    }

    // configure the elevator motor spark

    private void configCoralMotor() {
        SparkMaxConfig CoralConfig = new SparkMaxConfig();

        // config.encoder.positionConversionFactor(Math.PI * elevator_gearDiameter /
        // elevator_gearRatio);
        CoralConfig.encoder.positionConversionFactor(1);
        CoralConfig.inverted(true);
        CoralConfig.softLimit.forwardSoftLimitEnabled(false);
        CoralConfig.softLimit.reverseSoftLimit(0);
        CoralConfig.softLimit.reverseSoftLimitEnabled(false);
        CoralConfig.idleMode(IdleMode.kBrake);
        CoralConfig.openLoopRampRate(0.15);

        CoralConfig.closedLoop.maxOutput(1.0);
        CoralConfig.closedLoop.minOutput(-1.0);

        CoralConfig.closedLoopRampRate(0.15);
        CoralConfig.voltageCompensation(9.0);
        //// Down / outVelocity Values
        CoralConfig.closedLoop.maxMotion.maxAcceleration(5000, CORAL_CLOSED_LOOP_SLOT_DOWN);
        CoralConfig.closedLoop.maxMotion.maxVelocity(5000, CORAL_CLOSED_LOOP_SLOT_DOWN);
        CoralConfig.closedLoop.maxMotion.allowedClosedLoopError(coralPosTol, CORAL_CLOSED_LOOP_SLOT_DOWN);
        CoralConfig.closedLoop.pidf(0.4, 0.0, 0.0, 0.0, CORAL_CLOSED_LOOP_SLOT_DOWN);

        //// Up / in Velocity Values
        CoralConfig.closedLoop.maxMotion.maxAcceleration(5000, CORAL_CLOSED_LOOP_SLOT_UP);
        CoralConfig.closedLoop.maxMotion.maxVelocity(2000, CORAL_CLOSED_LOOP_SLOT_UP);
        CoralConfig.closedLoop.maxMotion.allowedClosedLoopError(coralPosTol, CORAL_CLOSED_LOOP_SLOT_UP);
        CoralConfig.closedLoop.pidf(0.4, 0.0, 0.0, 0.0, CORAL_CLOSED_LOOP_SLOT_UP);

        CoralConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // config.smartCurrentLimit(50);
        CoralConfig.smartCurrentLimit(20, 50);

        coralMotor.configure(CoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double calcCoralCompensation(double armPosDeg) {
        // calcuate the new coral position based strictly on the arm angle
        // y = new coral position based on the extreeme values of coral and arm angle
        // x = incomming arm angle
        return ((coral_m * armPosDeg) + coral_b);
    }

    public void setCoralPhase(CoralPhase newPhase) {
        if (currCoralPhase == CoralPhase.CLIMBENABLED) {
            return;
        }
        currCoralPhase = newPhase;


        // These are places where motor power is set instead of positional reference
        switch (currCoralPhase) {
            case INIT:
                // Not sure what to put here but I reserved it a space
                break;

            case PRECORAL:
                coralMotor.set(SPEED_PRECORAL);
                break;

            case LEVEL_1_DEPLOY:
                coralMotor.set(SPEED_LEVEL1_DEPLOY);
                break;

            case LEVEL_2_DEPLOY:
                coralMotor.set(SPEED_LEVEL2_DEPLOY);
                break;

            case LEVEL_3_DEPLOY:
                coralMotor.set(SPEED_LEVEL3_DEPLOY);
                break;
            case LEVEL_4_DEPLOY:
                coralMotor.set(SPEED_LEVEL4_DEPLOY);
                break;

            case ALGE_EXTRACT:
                coralMotor.set(SPEED_ALGE_EXTRACT);
                break;
            case STOP:
                coralMotor.set(0);
                break;
            case CLIMBENABLED:
                coralMotor.set(0);
                break;
            default:
                // These are where we set motor Power
        }
    }

    public CoralPhase getCoralPhase() {
        return currCoralPhase;
    }

    /*
     * public void setMandB(double new_m, double new_b) {
     * coral_m = new_m;
     * coral_b = new_b;
     * }
     */

    public Boolean getLowerSensor() {
        return !Coralhopper.get();
    }

    public Boolean getUpperSensor() {
        return !CoralIntake.get();
    }

    public enum CoralPhase {
        INIT,
        PRECORAL,
        // LOOKING_UPPER_SENSOR,
        // TRIPPED_UPPER_SENSOR,
        // LOOKING_LOWER_SENSOR,
        // TRIPPED_LOWER_SENSOR,
        CORAL_INDEX_WAITING,
        // CORAL_INDEX_COMPLETE,
        FINAL_POSITIONING,
        HOLDING,
        LEVEL_1_DEPLOY,
        LEVEL_2_DEPLOY,
        LEVEL_3_DEPLOY,
        LEVEL_4_DEPLOY,
        ALGE_EXTRACT,
        STOP,
        CLIMBENABLED

    }

}
