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

public class AlgaeIntake extends SubsystemBase {
    // Elevator Config Parameters
    public SparkMax pivotMotor = new SparkMax(CanIds.ALGAE_PIVOT_MOTOR, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(CanIds.ALGAE_INTAKE_MOTOR, MotorType.kBrushless);

    private double pivot_gearRatio = (2.89 * 5.23 * 5.23);
    private double pivotShaftDiameter = 0.75;
    private double pivotCurPos = 0.0;
    private double pivotCmdPos = PivotPos.START.pivotAngle;
    private final double pivotPosTol = 3;

    private PivotPos targetPos = PivotPos.START;
    private Status intakeStatus = Status.STOPPED;

    private final ClosedLoopSlot PIVOT_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot PIVOT_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_UP;

    public AlgaeIntake() {
        configIntakeMotor();
        configPivotMotor();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        pivotCurPos = pivotMotor.getEncoder().getPosition();

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

        pivotMotor.getClosedLoopController().setIAccum(0);
        intakeMotor.getClosedLoopController().setIAccum(0);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setNewPos(PivotPos tpos) {
        // Need to insert safety logic here
        targetPos = tpos;
        if (tpos == PivotPos.STOP) {
            setPivotCmdPos(getPivotCurPos());
        } else {
            setPivotCmdPos(tpos.getPivotPos());
        }

    }

    public void setIntakeStatus(Status status) {
        intakeStatus = status;

        intakeMotor.set(status.getIntakeSpeed());

    }

    // expose the current position
    public double getPivotCurPos() {
        return (pivotCurPos);
    }

    // Set the new ArmCommandPos
    private void setPivotCmdPos(double newPos) {
        this.pivotCmdPos = newPos;
        if (newPos > pivotCurPos) {
            PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_UP;
        } else {
            PivotCurrentSlot = PIVOT_CLOSED_LOOP_SLOT_DOWN;
        }
        pivotMotor.getClosedLoopController().setReference(newPos,
                ControlType.kMAXMotionPositionControl, PivotCurrentSlot);
    }

    public boolean isPivotAtTarget(PivotPos tpos) {

        return (CommonLogic.isInRange(getPivotCurPos(), tpos.getPivotPos(), pivotPosTol));
    }

    // configure the elevator motor spark
    private void configIntakeMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // config.softLimit.forwardSoftLimit(100);
        config.softLimit.forwardSoftLimitEnabled(false);
        // config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.idleMode(IdleMode.kBrake);
        //// Down Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(1);
        config.closedLoop.maxMotion.maxVelocity(1000);
        config.closedLoop.pidf(.004, 0.0, 0.0, 0.0);

        //// Up Velocity Values
       /* config.closedLoop.maxMotion.maxAcceleration(1);
        config.closedLoop.maxMotion.maxVelocity(1000);
        config.closedLoop.pidf(.004, 0.0, 0.0, 0.0);
*/
        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void configPivotMotor() {
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(1);
        config.inverted(true);
        config.softLimit.forwardSoftLimit(0);
        config.softLimit.forwardSoftLimitEnabled(false);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.idleMode(IdleMode.kBrake);
        //// Down Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(20000, PIVOT_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.maxVelocity(6000, PIVOT_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.pidf(.17, 0.0, 0.0, 0.0, PIVOT_CLOSED_LOOP_SLOT_DOWN);

        //// Up Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(20000, PIVOT_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.maxVelocity(6000, PIVOT_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.pidf(.17, 0.0, 0.0, 0.0, PIVOT_CLOSED_LOOP_SLOT_UP);

        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);

        /*
         * AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
         * absEncConfig.zeroOffset(0.649);
         * absEncConfig.inverted(false);
         * absEncConfig.positionConversionFactor(360);
         * 
         * config.absoluteEncoder.apply(absEncConfig);
         */
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public enum Status {
        IN(0.25),
        OUT(-0.25),
        HOLDING(0),
        STOPPED(0);

        private final double IntakeSpeed;

        Status(double IntakeSpeed) {
            this.IntakeSpeed = IntakeSpeed;
        }

        public double getIntakeSpeed() {
            return IntakeSpeed;
        }
    }

    public enum PivotPos {
        START(0),
        PICKUP(270),
        HELD(100),
        ALLTHEWAYOUT(380),
        OUTOFWAY(70),
        STOP(0);

        private final double pivotAngle;

        PivotPos(double pivotAngle) {
            this.pivotAngle = pivotAngle;
        }

        public double getPivotPos() {
            return pivotAngle;
        }

    }

    public double getTargetArmPos() {
        return pivotCmdPos;
    }

}
