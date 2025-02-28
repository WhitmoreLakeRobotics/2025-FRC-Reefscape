package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.commands.*;
import frc.utils.CommonLogic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Coral.CoralPhase;


public class ElevatorAndArm extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax elevatorMotor = new SparkMax(CanIds.ELEVATOR_MOTOR, MotorType.kBrushless);
    private SparkMax armMotor = new SparkMax(CanIds.ARM_MOTOR, MotorType.kBrushless);
    //private SparkMax coralMotor = new SparkMax(CanIds.CORAL_MOTOR, MotorType.kBrushless);

    private double elevator_gearRatio = (5 / 1);
    private double elevator_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;
    private double elevatorCurPos = 0.0;
    private double elevatorCmdPos = ElevAndArmPos.PICKUP.elevPos;
    private final double elevatorPosTol = 0.25;

    private final double armPosTol = 0.25;
    private double arm_gearRatio = (2.89 * 3.61 * 74 / 14);
    private double arm_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;

    private double armCurPos = 0.0;
    private double armCmdPos = ElevAndArmPos.PICKUP.armPos;
    private double armDirection = 0;

    private double Coral_m = 0;
    private double Coral_b = 0;
    private Coral m_coral = null;

    private ElevAndArmPos targetPos = ElevAndArmPos.PICKUP;

    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_UP;

    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot ArmCurrentSlot = ARM_CLOSED_LOOP_SLOT_UP;

    private boolean isBrake = true;

    public boolean isIntaking = false;

    public ElevatorAndArm() {
        configElevatorMotor();
        configArmMotor();

    }

    public void setCoralSystem (Coral newCoral){
        m_coral = newCoral;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        elevatorCurPos = elevatorMotor.getEncoder().getPosition();
        armCurPos = armMotor.getAbsoluteEncoder().getPosition();

        // Arm direction is positive when cmdPos is greater than curPos
        armDirection = Math.signum(armCmdPos - armCurPos);

        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.



        if (isArmAtTarget(ElevAndArmPos.SAFETYPOS)) {
            // System.err.print("Safety logic" + (targetPos.armPos >
            // ElevAndArmPos.SAFETYPOS.armPos)+ " "+ isArmAtTarget(ElevAndArmPos.SAFETYPOS)
            // + " " + (armCurPos > ElevAndArmPos.SAFETYPOS.armPos)) ;
        }

        if (!isElevatorAndArmAtTarget(targetPos)) {
            if ((targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos) && (armCurPos < ElevAndArmPos.SAFETYPOS.armPos)) {
                setElevatorAndArmPos(ElevAndArmPos.SAFETYPOS);
            } else if ((targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos)
                    && ((isArmAtTarget(ElevAndArmPos.SAFETYPOS)) || (armCurPos > ElevAndArmPos.SAFETYPOS.armPos))) {

                setElevatorAndArmPos(targetPos);

            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && !isElevatorAtTarget(ElevAndArmPos.SAFETYPOS)) {
                setElevatorAndArmPos(ElevAndArmPos.SAFETYPOS);
            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && isElevatorAtTarget(ElevAndArmPos.SAFETYPOS)) {
                setElevatorAndArmPos(targetPos);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {

        elevatorMotor.getClosedLoopController().setIAccum(0);
        armMotor.getClosedLoopController().setIAccum(0);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setNewPos(ElevAndArmPos tpos) {
        // Need to insert safety logic here
        //if (!isIntaking) {
            targetPos = tpos;

            setElevatorCmdPos(tpos.getElevPos());
            setArmCmdPos(tpos.getArmPos());

            switch (targetPos) {
                case ALGAEEXTRACTLOWER:
                case ALGAEEXTRACTUPPER:
                    m_coral.setCoralPhase(CoralPhase.ALGE_EXTRACT);
                    break;
                case PICKUP:
                    m_coral.setCoralPhase(CoralPhase.PRECORAL);



                default:
                    break;
            }
      //  }
    }

    public void setAlgaeExtract(ElevAndArmPos tpos) {
        // Need to insert safety logic here
        targetPos = tpos;

        setElevatorCmdPos(tpos.getElevPos());
        setArmCmdPos(tpos.getArmPos());
    }

    // expose the current position
    public double getElevatorCurPos() {
        return (elevatorCurPos);
    }

    public double getElevatorTargPos() {
        return (elevatorCmdPos);
    }

    public ElevAndArmPos getTargetPos() {
        return targetPos;
    }


    private void setElevatorCmdPos(double newPos) {
        elevatorCmdPos = newPos;
        if (newPos > elevatorCurPos) {
            ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_UP;
        } else {
            ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_DOWN;
        }
        elevatorMotor.getClosedLoopController().setReference(newPos, ControlType.kPosition, ElevatorCurrentSlot);
    }

    private void setElevatorAndArmPos(ElevAndArmPos tpos) {
        setElevatorCmdPos(tpos.getElevPos());
        setArmCmdPos(tpos.getArmPos());

    }

    public void stopCmd() {

        setElevatorCmdPos(elevatorCurPos);
        setArmCmdPos(armCurPos);

    }

    public void coastCmd() {

        if (isBrake) {

            // make all moters coast else make them brake.

        }

    }

    // expose the current position
    public double getArmCurPos() {
        return (armCurPos);
    }


    // Set the new ArmCommandPos
    private void setArmCmdPos(double newPos) {
        this.armCmdPos = newPos;
        if (newPos > armCurPos) {
            ArmCurrentSlot = ARM_CLOSED_LOOP_SLOT_UP;
        } else {
            ArmCurrentSlot = ARM_CLOSED_LOOP_SLOT_DOWN;
        }
        armMotor.getClosedLoopController().setReference(newPos,
                ControlType.kPosition, ArmCurrentSlot);
    }


    public boolean isElevatorAtTarget(ElevAndArmPos tpos) {

        return (CommonLogic.isInRange(getElevatorCurPos(), tpos.elevPos, 2 * elevatorPosTol));
    }

    public boolean isArmAtTarget(ElevAndArmPos tpos) {

        return (CommonLogic.isInRange(armCurPos, tpos.armPos, 3));
    }


    public boolean isElevatorAndArmAtTarget(ElevAndArmPos tpos) {

        return (isElevatorAtTarget(tpos) && isArmAtTarget(tpos));
    }


    // configure the elevator motor spark
    private void configElevatorMotor() {
        SparkMaxConfig Elevconfig = new SparkMaxConfig();

        // config.encoder.positionConversionFactor(Math.PI * elevator_gearDiameter /
        // elevator_gearRatio);
        Elevconfig.encoder.positionConversionFactor(1);
        Elevconfig.inverted(true);
        Elevconfig.softLimit.forwardSoftLimit(66);
        Elevconfig.softLimit.forwardSoftLimitEnabled(true);
        Elevconfig.softLimit.reverseSoftLimit(0);
        Elevconfig.softLimit.reverseSoftLimitEnabled(true);
        Elevconfig.idleMode(IdleMode.kBrake);
        //// Down Velocity Values
        Elevconfig.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol);

        Elevconfig.closedLoop.maxMotion.maxAcceleration(2500, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        Elevconfig.closedLoop.maxMotion.maxVelocity(1000, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        Elevconfig.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        Elevconfig.closedLoop.pidf(.08, 0.0, 0.0, 0.0, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);

        //// Up Velocity Values
        Elevconfig.closedLoop.maxMotion.maxAcceleration(5000, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        Elevconfig.closedLoop.maxMotion.maxVelocity(2000, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        Elevconfig.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        Elevconfig.closedLoop.pidf(0.5, 0.0, 0.0, 0.0, ELEVATOR_CLOSED_LOOP_SLOT_UP);

        Elevconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // config.smartCurrentLimit(50);
        Elevconfig.smartCurrentLimit(35, 50);

        elevatorMotor.configure(Elevconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configArmMotor() {

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig.softLimit.forwardSoftLimit(ElevAndArmPos.OUTOFWAY.armPos);
        armConfig.softLimit.forwardSoftLimitEnabled(true);
        armConfig.softLimit.reverseSoftLimit(ElevAndArmPos.PICKUP.armPos);
        armConfig.softLimit.reverseSoftLimitEnabled(true);
        armConfig.closedLoop.maxMotion.allowedClosedLoopError(armPosTol);
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(false);
        //// Down Velocity Values
        armConfig.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_DOWN);
        armConfig.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_DOWN);
        armConfig.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_DOWN);
        armConfig.closedLoop.pidf(.009, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_DOWN);
        //// Up Velocity Values
        armConfig.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.pidf(.027, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_UP);

        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // config.smartCurrentLimit(50);
        armConfig.smartCurrentLimit(50, 50);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig.zeroOffset(0.649);
        absEncConfig.inverted(false);
        absEncConfig.positionConversionFactor(360);

        armConfig.absoluteEncoder.apply(absEncConfig);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void configCoralCompensation() {
        // calculate m and b for y = mx + b linear formula
        /*
         * Position 1 is the CHold position
         * Position 2 is the Level 4 position
         *
         *                  Y2 - Y1
         * m = Coral_m = -------------
         *                  X2 - X1
         */

        Coral_m = (m_coral.CORAL_LEVLE4_POS - (m_coral.CORAL_PICKUP_POS)) /
                (ElevAndArmPos.LEVEL4.armPos - ElevAndArmPos.PICKUP.armPos);

        /*
         * b = Coral_b = Y1 - (m * X1)
         */

        Coral_b = (m_coral.CORAL_PICKUP_POS) - (Coral_m * ElevAndArmPos.PICKUP.armPos);

        m_coral.setM(Coral_m);
        m_coral.setB(Coral_b);

    }

    public enum ElevAndArmPos {
        PICKUP(24, 0),
        // START(24, 0),
        SAFETYPOS(40, 0),
        LEVEL1(62, 20),
        //LEVEL1DEL(62, 20),
        LEVEL2(51, 40),
        //LEVEL2DEL(51, 40),
        LEVEL3(166, 7), // was 1.2
        //LEVEL3DEL(166, 7),
        LEVEL4(166, 65.4), // 1.7
        //LEVEL4DEL(166, 65.4),
        ELVMAX(40, 65.9),
        ALGAEEXTRACTLOWER(70, 20),
        ALGAEEXTRACTUPPER(66, 58),
        OUTOFWAY(250, 0);
        //CIntake(24, 0), // was 2.5
        //CHold(24, 0),

        //CDeliver(24, 0),
        //CReturn(24, 0);

        private final double armPos;
        private final double elevPos;


        ElevAndArmPos(double armPos, double elevPos) {
            this.armPos = armPos;
            this.elevPos = elevPos;
        }

        public double getArmPos() {
            return armPos;
        }

        public double getElevPos() {
            return elevPos;
        }


    }

    public double getTargetArmPos() {
        return armCmdPos;
    }

}
