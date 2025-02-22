package frc.robot.subsystems;

import frc.robot.RobotContainer;
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
    private SparkMax deployMotor = new SparkMax(CanIds.BOTTOM_LIFT_MOTOR, MotorType.kBrushless);

    private double climbCurPos = 0.0;
    private final double deployPosTol = 5;
    private double deployCurPos = 0;


    private final ClosedLoopSlot CLIMB_CLOSED_LOOP_SLOT_UP = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot CLIMB_CLOSED_LOOP_SLOT_DOWN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot ClimbCurrentSlot = CLIMB_CLOSED_LOOP_SLOT_UP;

    private boolean bClimbEnabled = false;
    private double climbPower = 0.7;
    public Climb() {
        configClimbMotor();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        climbCurPos = climbMotor.getEncoder().getPosition();
        if (bClimbEnabled){
            updateClimbMotor();
        } 
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

    public void enableClimb() {
        bClimbEnabled = true;

    }

    // expose the current position
    public double getDeployCurPos() {
        return (deployCurPos);
    }

    private void updateClimbMotor() {
        climbMotor.set(climbPower * RobotContainer.getInstance().getArticulator().getLeftY());
    }

    // Set the new ArmCommandPos
    
    public boolean isDeployAtTarget(DeployPos tpos) {

        return (CommonLogic.isInRange(getDeployCurPos(), tpos.getDeployPos(), deployPosTol));
    }


    public void deployClimb(){
        deployMotor.getClosedLoopController().setReference(DeployPos.DEPLOY.getDeployPos(),ControlType.kPosition,CLIMB_CLOSED_LOOP_SLOT_UP);

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
        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(30, 40);

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
    private void configDeployMotor() {
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(1);
        config.inverted(true);
        config.softLimit.forwardSoftLimit(130);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(-1);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        //// In Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(30000, CLIMB_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.maxMotion.maxVelocity(6000, CLIMB_CLOSED_LOOP_SLOT_DOWN);
        config.closedLoop.pidf(.04, 0.0, 0.0004, 0.0, CLIMB_CLOSED_LOOP_SLOT_DOWN);

        //// Out Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(30000, CLIMB_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.maxMotion.maxVelocity(6000, CLIMB_CLOSED_LOOP_SLOT_UP);
        config.closedLoop.pidf(.08, 0.0, 0.0008, 0.0, CLIMB_CLOSED_LOOP_SLOT_DOWN);

        // config.smartCurrentLimit(50);
        config.smartCurrentLimit(30, 40);

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

    public enum DeployPos {
        START(0),
        DEPLOY(82),
        STOP(0);

        private final double deployAngle;

        DeployPos(double deployAngle) {
            this.deployAngle = deployAngle;
        }

        public double getDeployPos() {
            return deployAngle;
        }

    }


}
