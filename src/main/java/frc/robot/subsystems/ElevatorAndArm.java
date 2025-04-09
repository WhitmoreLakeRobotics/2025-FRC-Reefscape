package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.Constants.CanIds;
import frc.robot.commands.*;
import frc.utils.CommonLogic;
import frc.utils.RobotMath;
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
import frc.robot.subsystems.Coral.CoralPhase;

public class ElevatorAndArm extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax elevatorMotor = new SparkMax(CanIds.ELEVATOR_MOTOR, MotorType.kBrushless);
    private SparkMax armMotor = new SparkMax(CanIds.ARM_MOTOR, MotorType.kBrushless);
    // private SparkMax coralMotor = new SparkMax(CanIds.CORAL_MOTOR,
    // MotorType.kBrushless);

    private double elevator_gearRatio = (5 / 1);
    private double elevator_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;
    private double elevatorCurPos = 0.0;
    private double elevatorCmdPos = ElevAndArmPos.START.elevPos;
    private final double elevatorPosTol = 0.25;

    private final double armPosTol = 0.25;
    private double arm_gearRatio = (2.89 * 3.61 * 74 / 14);
    private double arm_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;

    private double armCurPos = 0.0;
    private double armCmdPos = ElevAndArmPos.START.armPos;
    private double armDirection = 0;

    private double Coral_m = 0;
    private double Coral_b = 0;
    private Coral m_coral = null;

    private ElevAndArmPos targetPos = ElevAndArmPos.START;

    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_THROWING = ClosedLoopSlot.kSlot3;

    private ClosedLoopSlot ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_UP;

    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_THROWING = ClosedLoopSlot.kSlot3;

    private ClosedLoopSlot ArmCurrentSlot = ARM_CLOSED_LOOP_SLOT_UP;
    public ALGAE_THROWING_STAGE currAlgaeThrowingStage = ALGAE_THROWING_STAGE.OTHER_ACTIONS;

    private boolean isBrake = true;
    private double disableIntakingTime = 0;

    public boolean isIntaking = false;

    public ElevatorAndArm() {
        configElevatorMotor();
        configArmMotor();

    }

    public void setCoralSystem(Coral newCoral) {
        m_coral = newCoral;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        elevatorCurPos = elevatorMotor.getEncoder().getPosition();
        armCurPos = armMotor.getAbsoluteEncoder().getPosition();

        // Arm direction is positive when cmdPos is greater than curPos
        armDirection = Math.signum(armCmdPos - armCurPos);


        switch (currAlgaeThrowingStage) {

            case HOLDING_ALGAE:
                break;


            case ALGAE_THROWING_PREP:
                // optimize by getting into throwing pos before we move to the barge.
                break;

            case ALGAE_THROWING_START:
                // Set this to start the throwing  squence
                // use external command to throw the ALGAE and enter this
                if (isElevatorAndArmAtTarget(ElevAndArmPos.ALGAE_THROWING_STARTPOS)){
                    currAlgaeThrowingStage = ALGAE_THROWING_STAGE.ALGAE_THROWING_RELEASE;
                    setArmCmdPos(ElevAndArmPos.ALGAE_THROWING_FINISHPOS.armPos, ARM_CLOSED_LOOP_SLOT_THROWING);
                    setElevatorCmdPos(ElevAndArmPos.ALGAE_THROWING_FINISHPOS.elevPos,ELEVATOR_CLOSED_LOOP_SLOT_THROWING);
                    targetPos = ElevAndArmPos.ALGAE_THROWING_FINISHPOS;
                }

                break;

            case ALGAE_THROWING_RELEASE:
                if (armCurPos >= ElevAndArmPos.ALGAE_THROWING_RELEASE_POS.armPos) {
                    m_coral.setCoralPhase(CoralPhase.ALGAE_DEPLOY_BARGE);
                    currAlgaeThrowingStage = ALGAE_THROWING_STAGE.ALGAE_THROWING_FINISH;
                }
                break;

            case ALGAE_THROWING_FINISH:
                // We get to the finish position and are able to stop there
                if (isElevatorAndArmAtTarget(ElevAndArmPos.ALGAE_THROWING_FINISHPOS)){
                    currAlgaeThrowingStage = ALGAE_THROWING_STAGE.OTHER_ACTIONS;
                    m_coral.setCoralPhase(CoralPhase.STOP);
                    setNewPos(ElevAndArmPos.SAFETYPOS);
                }

                // We overshot the arm pos and just need to stop the ossiclations
                if (armCurPos > ElevAndArmPos.ALGAE_THROWING_FINISHPOS.armPos){
                    //currAlgaeThrowingStage = ALGAE_THROWING_STAGE.OTHER_ACTIONS;
                    m_coral.setCoralPhase(CoralPhase.STOP);
                    setArmCmdPos(ElevAndArmPos.ALGAE_THROWING_FINISHPOS.armPos);
                }

                // We overshot the Elevator pos and just need to stop the ossiclations
                if (elevatorCurPos > ElevAndArmPos.ALGAE_THROWING_FINISHPOS.elevPos){
                    //currAlgaeThrowingStage = ALGAE_THROWING_STAGE.OTHER_ACTIONS;
                    m_coral.setCoralPhase(CoralPhase.STOP);
                    setElevatorCmdPos(ElevAndArmPos.ALGAE_THROWING_FINISHPOS.elevPos);
                }

                break;


            case OTHER_ACTIONS:
                periodic_other_actions ();
                break;

            default:
                break;

        }
    }

    public void setAlgaeThrowingPhase (ALGAE_THROWING_STAGE newStage){
        currAlgaeThrowingStage = newStage;
    }
    private void periodic_other_actions () {

        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.

        if ((RobotMath.getTime() > disableIntakingTime) && (isIntaking == true)) {
            isIntaking = false;
        }

        if (isArmAtTarget(ElevAndArmPos.SAFETYPOS)) {
            // System.err.print("Safety logic" + (targetPos.armPos >
            // ElevAndArmPos.SAFETYPOS.armPos)+ " "+ isArmAtTarget(ElevAndArmPos.SAFETYPOS)
            // + " " + (armCurPos > ElevAndArmPos.SAFETYPOS.armPos)) ;
        }

        if (!isElevatorAndArmAtTarget(targetPos)) {
            if (((targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos) && (armCurPos <= ElevAndArmPos.SAFETYPOS.armPos))
            && (!isElevatorAndArmAtTarget(ElevAndArmPos.SAFETYPOS))) {
                setElevatorAndArmPos(ElevAndArmPos.SAFETYPOS);
                //System.err.println("First Saftey Check");
            } else if ((targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos)
                    && ((isArmAtTarget(ElevAndArmPos.SAFETYPOS)) || (armCurPos >= ElevAndArmPos.SAFETYPOS.armPos))) {
                       // System.err.println("Second Saftey Check");

                setElevatorAndArmPos(targetPos);

            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && !isElevatorAtTarget(ElevAndArmPos.SAFETYPOS)) {
                       // System.err.println("Third Saftey Check");

                setElevatorAndArmPos(ElevAndArmPos.SAFETYPOS);
            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && isElevatorAtTarget(ElevAndArmPos.SAFETYPOS)) {
                        //System.err.println("Fourth Saftey Check");

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

    public void setBlockMoves(boolean newBlockValue) {
        isIntaking = newBlockValue;
        if (isIntaking) {
            // in 5 seconds re-enable movement of the arm
            disableIntakingTime = RobotMath.getTime() + 2.50;
        }

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setNewPos(ElevAndArmPos tpos) {
        // Need to insert safety logic here
        if (isIntaking) {
            System.err.println("E&A setNewPos is blocked by intaking");
            return;
        }
        targetPos = tpos;

        setArmCmdPos(targetPos.armPos);
        setElevatorCmdPos(targetPos.elevPos);



        switch (targetPos) {
            case ALGAEEXTRACTLOWER:
            case ALGAEEXTRACTUPPER:
                m_coral.setCoralPhase(CoralPhase.ALGAE_EXTRACT);
                m_coral.stopFunnel();
                break;

            case PICKUP:
                m_coral.setCoralPhase(CoralPhase.WAITING_4_PICKUP);
                break;

            case ALGAE_THROWING_STARTPOS:
                currAlgaeThrowingStage =ALGAE_THROWING_STAGE.ALGAE_THROWING_PREP;
                break;

            default:
                // I am not sure if we want to go to holding for all other positions
                // m_coral.setCoralPhase(CoralPhase.HOLDING);
                m_coral.stopFunnel();
                break;
        }

    }
    public void autonInit() {
        isIntaking = false;
    }

    public void setAlgaeExtract(ElevAndArmPos tpos) {
        // Need to insert safety logic here
        setNewPos (tpos);
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
        //elevatorCmdPos = newPos;
        ClosedLoopSlot slot;
        if (newPos > elevatorCurPos) {
            slot = ELEVATOR_CLOSED_LOOP_SLOT_UP;
        } else {
            slot = ELEVATOR_CLOSED_LOOP_SLOT_DOWN;
        }
        //elevatorMotor.getClosedLoopController().setReference(newPos, ControlType.kPosition, ElevatorCurrentSlot);
        setElevatorCmdPos(newPos, slot);
    }


    private void setElevatorCmdPos(double newPos, ClosedLoopSlot cmdSlot) {

        elevatorCmdPos = newPos;
        ElevatorCurrentSlot = cmdSlot;
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
        // this.armCmdPos = newPos;
        ClosedLoopSlot slot;
        if (newPos > armCurPos) {
            slot = ARM_CLOSED_LOOP_SLOT_UP;
        } else {
            slot = ARM_CLOSED_LOOP_SLOT_DOWN;
        }
        setArmCmdPos(newPos, slot);
    }

    private void setArmCmdPos(double newPos, ClosedLoopSlot cmdSlot) {
        this.armCmdPos = newPos;
        ArmCurrentSlot = cmdSlot;

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

        //// Throwing Algae Velocity Values
        Elevconfig.closedLoop.maxMotion.maxAcceleration(5000, ELEVATOR_CLOSED_LOOP_SLOT_THROWING);
        Elevconfig.closedLoop.maxMotion.maxVelocity(2000, ELEVATOR_CLOSED_LOOP_SLOT_THROWING);
        Elevconfig.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol, ELEVATOR_CLOSED_LOOP_SLOT_THROWING);
        Elevconfig.closedLoop.pidf(0.5 * 1.5, 0.0, 0.0, 0.0, ELEVATOR_CLOSED_LOOP_SLOT_THROWING);

        Elevconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        // config.smartCurrentLimit(50);
        Elevconfig.smartCurrentLimit(60, 60);

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
        armConfig.closedLoop.pidf(.0125, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_DOWN); // was p = 0.009
        //// Up Velocity Values
        armConfig.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_UP);
        armConfig.closedLoop.pidf(.027, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_UP);

        //Throwing Alge Velocity Values
        armConfig.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_THROWING);
        armConfig.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_THROWING);
        armConfig.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_THROWING);
        armConfig.closedLoop.pidf(.027 * 1.5, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_THROWING);

        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        // config.smartCurrentLimit(50);
        armConfig.smartCurrentLimit(50, 50);

        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        //absEncConfig.zeroOffset(0.649);
        absEncConfig.inverted(false);
        absEncConfig.positionConversionFactor(360);

        armConfig.absoluteEncoder.apply(absEncConfig);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void enableInit() {
        elevatorMotor.getClosedLoopController().setReference(elevatorMotor.getEncoder().getPosition(),
                ControlType.kPosition);
        elevatorMotor.set(0);

        armMotor.getClosedLoopController().setReference(armMotor.getEncoder().getPosition(), ControlType.kPosition);
        armMotor.set(0);
    }

    public void configCoralCompensation() {
        // calculate m and b for y = mx + b linear formula
        /*
         * Position 1 is the CHold position
         * Position 2 is the Level 4 position
         *
         * Y2 - Y1
         * m = Coral_m = -------------
         * X2 - X1
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

    public enum ALGAE_THROWING_STAGE{
        HOLDING_ALGAE,
        ALGAE_THROWING_PREP,
        ALGAE_THROWING_START,
        ALGAE_THROWING_RELEASE,
        ALGAE_THROWING_FINISH,
        OTHER_ACTIONS
    }



    public enum ElevAndArmPos {
        PICKUP(23, 0),
        START(23, 0),
        SAFETYPOS(42, 0),
        LEVEL1(63, 6), // was 10
        // LEVEL1DEL(62, 20),
        LEVEL2(53, 22.2), //was 37
        // LEVEL2DEL(51, 40),
        LEVEL3(168, 2.4), // was 4
        // LEVEL3DEL(166, 7),
        LEVEL4(160, 39.24), // was 65.4
        LEVEL4OUTOFWAY(183,39.24),// was 65.4
        // LEVEL4DEL(166, 65.4),
        ELVMAX(40, 39.54),// was 65.9
        ALGAEEXTRACTLOWER(72, 12),// was 20
        ALGAEEXTRACTUPPER(68, 34.8),// was 58
        // The ALGAE Throwing are meant for the Barge
        ALGAE_THROWING_STARTPOS (42,20),
        ALGAE_THROWING_RELEASE_POS (130,30),
        ALGAE_THROWING_FINISHPOS (183,35),

        OUTOFWAY(235, 0);

        // CIntake(24, 0), // was 2.5
        // CHold(24, 0),

        // CDeliver(24, 0),
        // CReturn(24, 0);

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
