package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.CanIds;
import frc.robot.commands.drivebase.DriveToPickup.TARGETPOS;
import frc.robot.subsystems.Coral.CoralPhase;
import frc.utils.CommonLogic;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList; // Ensure this import is complete and used in the code
import java.util.List;
import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkBase.ResetMode;

public class DiverAssist extends SubsystemBase {
    // GLOBAL VARIABLES GO BELOW THIS LINE
    private DAStatus currStatus = DAStatus.INIT;
    private DriveTrain driveTrain;
    private Pose2d robotPose;
    private CoralPhase currCoralPhase;
    private Coral coral;


    

    
    // GLOBAL VARIABLES GO ABOVE THIS LINE
    // SYSTEM METHODS GO BELOW THIS LINE
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

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    // expose the current position
    public void autonInit() {


    }
    // SYSTEM METHODS GO ABOVE THIS LINE
    // PROCESSING METHODS GO BELOW THIS LINE
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

    // PROCESSING METHODS GO ABOVE THIS LINE
    // SUPPORTING METHODS GO BELOW THIS LINE

    

    public Targets[] getTargets(String type) {
    List<Targets> result = new ArrayList<Targets>();
    for (Targets targets : Targets.values()) {
        if (targets.getTargetType() == type) {
            result.add(targets);
        }
    }
    return result.toArray(new Targets[result.size()]);
}

    // SUPPORTING METHODS GO ABOVE THIS LINE
    // ENUMERATIONS GO BELOW THIS LINE 
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
    // ENUMERATIONS GO ABOVE THIS LINE
    // REFERENCE METHODS GO BELOW THIS LINE

public static double CapMotorPower(double MotorPower, double negCapValue, double posCapValue) {
    // logic to cap the motor power between a good range
    double retValue = MotorPower;

    if (MotorPower < negCapValue) {
      retValue = negCapValue;
    }

    if (MotorPower > posCapValue) {
      retValue = posCapValue;
    }

    return retValue;
  }

  public static final double joyDeadBand(double joy, double deadband) {

    double retValue = joy;
    if (Math.abs(retValue) < Math.abs(deadband)) {
      retValue = 0;
    }
    return Math.pow(retValue, 2) * Math.signum(joy);
  }

  public static final boolean isInRange(double curValue, double targetValue, double Tol) {

    double loVal = targetValue - Tol;
    double hiVal = targetValue + Tol;
    boolean retValue = false;

    if (curValue > loVal && curValue < hiVal) {
      retValue = true;
    }
    return retValue;
  }

  public static double getTimeInSeconds() {
    return (System.nanoTime() / Math.pow(10, 9)); // returns time in seconds
  }

  public double deg2Radians(double degrees) {
    return Math.toRadians(degrees);
  }

  public static double calcArcLength(double degrees, double radius) {
    return (Math.toRadians(degrees) * radius);
  }

  public static double gotoPosPIDF(double P, double F_hold, double currentPos, double targetPos) {
    double delta = targetPos - currentPos;

    return (delta * P) + F_hold;

  }

  public static double AutoTurnPIDF(double P, double F_hold, double currentPos, double targetPos) {
    double delta = targetPos - currentPos;
    if(delta > 180){
      delta = -delta;
    }

    return (delta * P) + F_hold;

  }

public static int getIsBlue() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            // is blue = false
            return 0;
        }
        if (ally.get() == Alliance.Blue) {
            // is blue = true
            return 1;
        }
    } else {
        // naptime
        return -1;
    }
        return -1;
}



public static double getTimeInMIlliseconds() {
    return (System.nanoTime() / Math.pow(10, 6));
}

public static double headingDeltaInDegree(double currentHeading, double targetHeading) {
    double headingDelta = 0;
    double invertedHeadingDelta = 0;

    // Positive value
    if (currentHeading >= 0 && targetHeading >= 0) {
        headingDelta = targetHeading - currentHeading;
    }
    // one of each
    else if (currentHeading >= 0 && targetHeading <= 0) {
        // headingDelta = (targetHeading + currentHeading);
        headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        invertedHeadingDelta = Math.abs(360 + targetHeading) - Math.abs(currentHeading);
        headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        if (invertedHeadingDelta != headingDelta) {
            headingDelta = headingDelta * -1;
        }
    }
    // one of each again
    else if (currentHeading <= 0 && targetHeading >= 0) {
        // headingDelta = -1 * (targetHeading + currentHeading);
        headingDelta = Math.abs(targetHeading) + Math.abs(currentHeading);
        invertedHeadingDelta = Math.abs(360 - targetHeading) - Math.abs(currentHeading);
        headingDelta = Math.min(Math.abs(headingDelta), Math.abs(invertedHeadingDelta));
        if (invertedHeadingDelta == headingDelta) {
            headingDelta = headingDelta * -1;
        }
    }
    // both negative
    else if (currentHeading <= 0 && targetHeading <= 0) {
        headingDelta = targetHeading - currentHeading;
    }
    return headingDelta;
}

public static double calcTurnRateAbs(double currentHeading, double targetHeading, double proportion) {

    double headingDelta = headingDeltaInDegree(currentHeading, targetHeading);

    double commandedTurnRate = headingDelta * proportion;
    return commandedTurnRate; // IS ALWAYS POSITIVE!
}

public static double metersToInches(double meter) {
    return meter / 0.0254;
}

public static double inchesToMeters(double inches) {
    return inches * 0.0254;
}

    // REFERENCE METHODS GO ABOVE THIS LINE

}
