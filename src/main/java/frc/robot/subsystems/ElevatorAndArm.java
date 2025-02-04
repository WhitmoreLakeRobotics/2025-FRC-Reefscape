package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.commands.*;
import frc.utils.CommonLogic;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class ElevatorAndArm extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax elevatorMotor = new SparkMax(CanIds.ELEVATOR_MOTOR, MotorType.kBrushless);
    private SparkMax armMotor = new SparkMax(CanIds.ARM_MOTOR, MotorType.kBrushless);

    private double elevator_gearRatio = (5 / 1);
    private double elevator_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;
    private double elevatorCurPos = 0.0;
    private double elevatorCmdPos = ElevAndArmPos.START.elevPos;
    private final double elevatorPosTol = 0.5;

    private final double armPosTol = 0.75;
    private double arm_gearRatio = (2.89 * 3.61 * 74 / 14);
    private double arm_gearDiameter = 1.685; // 14 tooth
    // https://www.andymark.com/products/35-series-symmetrical-hub-sprockets?via=Z2lkOi8vYW5keW1hcmsvV29ya2FyZWE6Ok5hdmlnYXRpb246OlNlYXJjaFJlc3VsdHMvJTdCJTIycSUyMiUzQSUyMjE0K3Rvb3RoK3Nwcm9ja2V0JTIyJTdE&Tooth%20Count=14%20(am-4790)&quantity=1;

    private double armCurPos = 0.0;
    private double armCmdPos = ElevAndArmPos.START.armPos;
    private double armDirection = 0;

    private ElevAndArmPos targetPos = ElevAndArmPos.START;

    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ELEVATOR_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_UP;

    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot ARM_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot ArmCurrentSlot = ARM_CLOSED_LOOP_SLOT_UP;

    public ElevatorAndArm() {
        configElevatorMotor();
        configArmMotor();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        elevatorCurPos = elevatorMotor.getEncoder().getPosition();

        armCurPos = armMotor.getAbsoluteEncoder().getPosition();

        // Arm direction is positive when cmdPos is greater than curPos
        armDirection = Math.signum(armCmdPos - armCurPos);
        /*
         * armMotor.getClosedLoopController().setReference(armCmdPos,
         * ControlType.kMAXMotionPositionControl, ArmCurrentSlot,
         * Math.abs(Math.sin(armCurPos)) * armDirection);
         */
        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.
       
       
       /*  if (!isElevatorAndArmAtTarget(targetPos)) {
            if (targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos && armCurPos < ElevAndArmPos.SAFETYPOS.armPos) {
                setArmCmdPos(ElevAndArmPos.SAFETYPOS.armPos);
            } else if (targetPos.armPos > ElevAndArmPos.SAFETYPOS.armPos
                    && isArmAtTarget(ElevAndArmPos.SAFETYPOS)) {
                setElevatorAndArmPos(targetPos);

            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && elevatorCurPos > ElevAndArmPos.SAFETYPOS.elevPos) {
                setElevatorAndArmPos(ElevAndArmPos.SAFETYPOS);
            } else if (targetPos.armPos < ElevAndArmPos.SAFETYPOS.armPos
                    && isElevatorAtTarget(ElevAndArmPos.SAFETYPOS)) {
                setElevatorAndArmPos(targetPos);
            }
        }*/
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

    private void setElevatorCmdPos(double newPos) {
        elevatorCmdPos = newPos;
        if (newPos > elevatorCurPos) {
            ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_UP;
        } else {
            ElevatorCurrentSlot = ELEVATOR_CLOSED_LOOP_SLOT_DOWN;
        }
        elevatorMotor.getClosedLoopController().setReference(newPos, ControlType.kPosition);
    }

    private void setElevatorAndArmPos(ElevAndArmPos tpos) {
        setElevatorCmdPos(tpos.getElevPos());
        setArmCmdPos(tpos.getArmPos());
    }

    // expose the current position
    public double getArmCurPos() {
        return (armCurPos);
    }

    // Set the new ArmCommandPos
    public void setArmCmdPos(double newPos) {
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

        return (CommonLogic.isInRange(getElevatorCurPos(), tpos.elevPos, elevatorPosTol));
    }

    public boolean isArmAtTarget(ElevAndArmPos tpos) {

        return (CommonLogic.isInRange(getArmCurPos(), tpos.armPos, armPosTol));
    }

    public boolean isElevatorAndArmAtTarget(ElevAndArmPos tpos) {

        return (isElevatorAtTarget(tpos) && isArmAtTarget(tpos));
    }

    // configure the elevator motor spark
    private void configElevatorMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        //config.encoder.positionConversionFactor(Math.PI * elevator_gearDiameter / elevator_gearRatio);
        config.encoder.positionConversionFactor(1);
        config.inverted(true);
        config.softLimit.forwardSoftLimit(100);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        //// Down Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(2500, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.maxVelocity(1000, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.pidf(.004, 0.0, 0.0, 0.0, ELEVATOR_CLOSED_LOOP_SLOT_DOWN);

        //// Up Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(2500, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.maxVelocity(1000, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.allowedClosedLoopError(elevatorPosTol, ELEVATOR_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.pidf(0.1, 0.0, 0.0, 0.0, ELEVATOR_CLOSED_LOOP_SLOT_UP);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

      //  config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);

        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configArmMotor() {

        SparkMaxConfig config = new SparkMaxConfig();

        config.softLimit.forwardSoftLimit(ElevAndArmPos.OUTOFWAY.armPos);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(ElevAndArmPos.PICKUP.armPos);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        config.inverted(false);
        //// Down Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.pidf(.008, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_DOWN);
        //// Up Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(25, ARM_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.maxVelocity(1000, ARM_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.allowedClosedLoopError(armPosTol, ARM_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.pidf(.02, 0.0, 0.0, 0.0, ARM_CLOSED_LOOP_SLOT_UP);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

      //  config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);
        
        AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
        absEncConfig.zeroOffset(0.649);
        absEncConfig.inverted(false);
        absEncConfig.positionConversionFactor(360);
        
        config.absoluteEncoder.apply(absEncConfig);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public enum ElevAndArmPos {
        PICKUP(22, 0),
        START(22, 0),
        SAFETYPOS(40, 0),
        LEVEL1(65, 5),
        LEVEL2(85, 10),
        LEVEL3(128, 15),
        LEVEL4(170, 17),
        ELVMAX(40, 20.5),
        OUTOFWAY(175, 0);

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
