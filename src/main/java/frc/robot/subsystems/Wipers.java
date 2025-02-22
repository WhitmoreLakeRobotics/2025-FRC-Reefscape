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

public class Wipers extends SubsystemBase {
    // Elevator Config Parameters
    public SparkMax leftMotor = new SparkMax(CanIds.RIGHT_GUIDE, MotorType.kBrushless);
    private SparkMax rightMotor = new SparkMax(CanIds.LEFT_GUIDE, MotorType.kBrushless);

    private double pivot_gearRatio = (2.89 * 5.23 * 5.23);
    private double pivotShaftDiameter = 0.75;
    private double leftCurPos = 0.0;
    private double rightCurrPos = 0.0;
    private double pivotCmdPos = GuidePos.START.guideAngle;
    private final double pivotPosTol = 0.1;

    private GuidePos rightTargetPos = GuidePos.START;
    private GuidePos leftTargetPos = GuidePos.START;

    private final ClosedLoopSlot GUIDE_CLOSED_LOOP_SLOT = ClosedLoopSlot.kSlot0;
    private ClosedLoopSlot GuideCurrentSlot = GUIDE_CLOSED_LOOP_SLOT;

    public Wipers() {
        configMotors();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        leftCurPos = leftMotor.getEncoder().getPosition();
        rightCurrPos = rightMotor.getEncoder().getPosition();

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

        rightMotor.getClosedLoopController().setIAccum(0);
        leftMotor.getClosedLoopController().setIAccum(0);
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    // expose the current position
    public double getLeftCurPos() {
        return leftCurPos;
    }
    public double getLeftTargetPos(){
        return leftTargetPos.guideAngle;
    }

    public double getRightTargetPos(){
        return rightTargetPos.guideAngle;
    }

    public double getRightCurPos() {
        return rightCurrPos;
    }

    // Set the new ArmCommandPos
    public void setLeftCmdPos(GuidePos newPos) {
        leftTargetPos = newPos;
       
        leftMotor.getClosedLoopController().setReference(newPos.guideAngle,
                ControlType.kPosition, GuideCurrentSlot);
    }

    public void setRightCmdPos(GuidePos newPos) {
        rightTargetPos = newPos;
       
        rightMotor.getClosedLoopController().setReference(newPos.guideAngle,
                ControlType.kPosition, GuideCurrentSlot);
    }


    public boolean isLeftAtTarget(GuidePos tpos) {

        return (CommonLogic.isInRange(getLeftCurPos(), tpos.getGuidePos(), 2*pivotPosTol));
    }

    public boolean isRightAtTarget(GuidePos tpos) {

        return (CommonLogic.isInRange(getRightCurPos(), tpos.getGuidePos(), 2*pivotPosTol));
    }

    public void leftRightOut() {
        setLeftCmdPos(GuidePos.OUT);
        setRightCmdPos(GuidePos.OUT);


    }
    // configure the elevator motor spark
   

    private void configMotors() {
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(1);
        config.inverted(false);
        config.softLimit.forwardSoftLimit(130);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(-1);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        //// In Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(30000, GUIDE_CLOSED_LOOP_SLOT);
        config.closedLoop.maxMotion.maxVelocity(6000, GUIDE_CLOSED_LOOP_SLOT);
        config.closedLoop.pidf(0.12, 0.0, 0.0, 0.0,GUIDE_CLOSED_LOOP_SLOT);

        //// Out Velocity Values
    

        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(10, 20);

        /*
         * AbsoluteEncoderConfig absEncConfig = new AbsoluteEncoderConfig();
         * absEncConfig.zeroOffset(0.649);
         * absEncConfig.inverted(false);
         * absEncConfig.positionConversionFactor(360);
         * 
         * config.absoluteEncoder.apply(absEncConfig);
         */
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.inverted(true);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public enum Wiper{
        LEFT, 
        RIGHT;

    }



    public enum GuidePos {
        START(1.2),
        OUT(6.7);

        private final double guideAngle;

        GuidePos(double pivotAngle) {
            this.guideAngle = pivotAngle;
        }

        public double getGuidePos() {
            return guideAngle;
        }

    }


}
