package frc.robot.subsystems;

import frc.robot.Constants.CanIds;
import frc.robot.commands.*;
import frc.robot.hardware.PidSlotPosition;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.commands.*;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.hardware.*;
public class Elevator extends SubsystemBase {
    PidSlotPosition slot0 = null;


    public Elevator() {
        SparkMax elevatorMotor = new SparkMax(CanIds.ELEVATOR_MOTOR, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();

        double gearRatio = 100 / 1;
        config.encoder.positionConversionFactor(Math.PI / gearRatio );
        config.softLimit.forwardSoftLimit(100);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.maxMotion.maxAcceleration(1,ClosedLoopSlot.kSlot0);
        config.closedLoop.maxMotion.maxVelocity(5000, ClosedLoopSlot.kSlot0);
        //config.closedLoop.maxMotion.maxJerk
        config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);
        
        elevatorMotor.configure(config, null, null);   
    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

