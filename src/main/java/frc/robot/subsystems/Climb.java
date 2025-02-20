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

public class Climb extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax climbMotor = new SparkMax(CanIds.CLIMB_MOTOR, MotorType.kBrushless);

    private double pivotCurPos = 0.0;
    private double pivotCmdPos = PivotPos.START.pivotAngle;
    private final double pivotPosTol = 5;


    private final ClosedLoopSlot PIVOT_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot PIVOT_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_UP;

    public Climb() {
        configClimbMotor();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pivotCurPos = climbMotor.getEncoder().getPosition();

        // Arm direction is positive when cmdPos is greater than curPos
        /*
         * armMotor.getClosedLoopController().setReference(armCmdPos,
         * ControlType.kMAXMotionPositionControl, ArmCurrentSlot,
         * Math.abs(Math.sin(armCurPos)) * armDirection);
         */
        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the
        // safety logic here.
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {

        climbMotor.getClosedLoopController().setIAccum(0);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setNewPos(PivotPos tpos) {
        // Need to insert safety logic here
      //  targetPos = tpos;
        if (tpos == PivotPos.STOP) {
            setPivotCmdPos(getPivotCurPos());
        } else {
            setPivotCmdPos(tpos.getPivotPos());
        }

    }

    // expose the current position
    public double getPivotCurPos() {
        return (pivotCurPos);
    }

    
    public void StopIntake(){
        setPivotCmdPos(pivotCurPos);
    }

    // Set the new ArmCommandPos
    private void setPivotCmdPos(double newPos) {
        this.pivotCmdPos = newPos;
        if (newPos > pivotCurPos) {
            PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_UP;
        } else {
            PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_DOWN;
        }
        climbMotor.getClosedLoopController().setReference(newPos,
                ControlType.kPosition, PivotCurrentSlot);
    }

    public boolean isPivotAtTarget(PivotPos tpos) {

        return (CommonLogic.isInRange(getPivotCurPos(), tpos.getPivotPos(), pivotPosTol));
    }


    private void configClimbMotor() {
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(1);
        config.inverted(true);
        config.softLimit.forwardSoftLimit(130);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(-1);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        //// In Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(30000, PIVOT_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.maxVelocity(6000, PIVOT_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.pidf(.04, 0.0, 0.0004, 0.0, PIVOT_CLOSED_LOOP_SLOT_DOWN);

        //// Out Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(30000, PIVOT_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.maxVelocity(6000, PIVOT_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.pidf(.08, 0.0, 0.0008, 0.0, PIVOT_CLOSED_LOOP_SLOT_UP);

        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(30, 30);

        /*
         * AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
         * absEncConfig.zeroOffset(0.649);
         * absEncConfig.inverted(false);
         * absEncConfig.positionConversionFactor(360);
         * 
         * config.absoluteEncoder.apply(absEncConfig);
         */
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


    public enum PivotPos {
        START(0),
        PICKUP(105),
        HELD(82),
        ALLTHEWAYOUT(123),
        CORALPICKUP(25),
        STOP(0);

        private final double pivotAngle;

        PivotPos(double pivotAngle) {
            this.pivotAngle = pivotAngle;
        }

        public double getPivotPos() {
            return pivotAngle;
        }

    }

    public double getTargetPivPos() {
        return pivotCmdPos;
    }

}
