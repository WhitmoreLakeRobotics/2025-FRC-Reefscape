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

public class CoralSeeder extends SubsystemBase {
    // Elevator Config Parameters
    private SparkMax coralMotor = new SparkMax(CanIds.CORAL_MOTOR, MotorType.kBrushless);
    

   
   
    private final ClosedLoopSlot CORAL_CLOSED_LOOP_SLOT_OUT = ClosedLoopSlot.kSlot0;
    private final ClosedLoopSlot CORAL_CLOSED_LOOP_SLOT_IN = ClosedLoopSlot.kSlot1;
    private ClosedLoopSlot CoralCurrentSlot = CORAL_CLOSED_LOOP_SLOT_OUT;

    public CoralSeeder() {
        configCoralMotor();


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
       

      
        // Probably should add some safety logic here
        // recommend storing new target position in a variable and then executing the safety logic here.
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void disablePeriodic() {
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    
    // expose the current position
    

   
   



   

   

   


   


    
   
    // configure the elevator motor spark
    private void configCoralMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder.positionConversionFactor(0);
        config.softLimit.forwardSoftLimit(100);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimit(0);
        config.softLimit.reverseSoftLimitEnabled(true);
        config.idleMode(IdleMode.kBrake);
        //// Down Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(1, CORAL_CLOSED_LOOP_SLOT_IN);
        config.closedLoop.maxMotion.maxVelocity(5000, CORAL_CLOSED_LOOP_SLOT_IN);
        config.closedLoop.pidf(.004, 0.0, 0.0, 0.5, CORAL_CLOSED_LOOP_SLOT_IN);

        //// Up Velocity Values
        config.closedLoop.maxMotion.maxAcceleration(1, CORAL_CLOSED_LOOP_SLOT_OUT);
        config.closedLoop.maxMotion.maxVelocity(5000, CORAL_CLOSED_LOOP_SLOT_OUT);
        config.closedLoop.pidf(.004, 0.0, 0.0, 0.5, CORAL_CLOSED_LOOP_SLOT_OUT);

        config.smartCurrentLimit(50);
        config.smartCurrentLimit(50, 50);
        
        coralMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

   

    
}
