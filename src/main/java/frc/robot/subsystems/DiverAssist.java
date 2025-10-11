package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.CanIds;
import frc.robot.commands.drivebase.DriveToPickup.TARGETPOS;
import frc.robot.subsystems.Coral.CoralPhase;
import frc.utils.CommonLogic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList; // Ensure this import is complete and used in the code
import java.util.List;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase.ResetMode;

public class DiverAssist extends SubsystemBase {
    
    private DAStatus currStatus = DAStatus.INIT;
    private DriveTrain driveTrain;
    private Pose2d robotPose;
    private CoralPhase currCoralPhase;
    private Coral coral;


    

    

    public DiverAssist() {
        

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch (currStatus)  {
            case RUNNING:
                getSubsystemState();
                break;
            case INIT:
                getSubsystems();
                getSubsystemState();
                currStatus = DAStatus.RUNNING;
                break;
        
            default:
                break;
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

        
    }

    private void getSubsystems() {
        driveTrain = RobotContainer.getInstance().m_driveTrain;

        coral = RobotContainer.getInstance().m_Coral;

    }

    private void getSubsystemState() {
        robotPose = driveTrain.getPose();

        currCoralPhase = coral.getCoralPhase();
        
    }

    private void determineCurrentAction() {
        // Determine what action we need to take based on the current state of the
        // subsystems

    }

    private void determinePossibleTargets(){
        // Determine what targets are closest based on the current state of the
        // subsystems.

    }

    private void determineBestTarget() {
        // Determine what target is best based on the current state of the subsystems.

    }

    private void setDriveTarget() {
        // Set the target for the drivebase based on the current state of the
        // subsystems.

    }

    private void determineLikelyActions() {
        // Determine what actions are possible based on the current state of the
        // subsystems.

    }

    private void executeAction() {
        // Execute the action determined by the current state of the subsystems.
        // This may be a simple command or a complex sequence of commands.
        // Need to set articulation and drivebase targets.
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // expose the current position
    public void autonInit() {


    }

    public Targets[] getTargets(String type) {
    List<Targets> result = new ArrayList<Targets>();
    for (Targets targets : Targets.values()) {
        if (targets.getTargetType() == type) {
            result.add(targets);
        }
    }
    return result.toArray(new Targets[result.size()]);
}


    public enum DAStatus {
        INIT, // Initialization is averything up to the point of where we are connected to our subsystems.
        RUNNING, // Cheaking the data and running it.
        STOP; // The end when connection is lost.

       // private final double guideAngle;
        /* 
        GuidePos(double pivotAngle) {
            this.guideAngle = pivotAngle;
        }
        
        public double getGuidePos() {
            return guideAngle;
        }
        */
    }

    public enum Targets {
        PICKUPLEFT(new Pose2d(6.0, 1.5, new Rotation2d(Math.toRadians(0.0))), "PICKUP"),
        PICKUPRIGHT(new Pose2d(6.0, 5.5, new Rotation2d(Math.toRadians(0.0))), "PICKUP"),
        ID8LEFT(new Pose2d(12.0, 1.5, new Rotation2d(Math.toRadians(0.0))), "DEPLOY"),
        ID8RIGHT(new Pose2d(12.0, 5.5, new Rotation2d(Math.toRadians(0.0))), "DEPLOY"),
        CLIMBLEFT(new Pose2d(18.0, 1.5, new Rotation2d(Math.toRadians(0.0))),"END");

        private final Pose2d targetPose;
        private final String targetType;
        Targets(Pose2d targetPose, String targetType) {
            this.targetPose = targetPose;
            this.targetType = targetType;

        }

        public Pose2d getTargetPose() {
            return targetPose;
        }

        public String getTargetType() {
            return targetType;
        }
    }

}
