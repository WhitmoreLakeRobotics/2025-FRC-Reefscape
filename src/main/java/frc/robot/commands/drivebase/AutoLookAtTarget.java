
package frc.robot.commands.drivebase;

import java.util.Currency;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.utils.CommonLogic;
import frc.utils.PID;

public class AutoLookAtTarget extends Command{
    Supplier<Optional<Pose2d>> poseSupplier;
    IntSupplier idSupplier;
    boolean left;
    DriveTrain swerve;
    double timestamp;
    boolean bdone = false;
    public Pose2d distFromTag;
    private PIDController xController = new PIDController(Constants.SwerveConstants.kPX, 0, 0);
    private PIDController yController = new PIDController(Constants.SwerveConstants.kPY, 0, 0);
    private PIDController thetaController = new PIDController(Constants.SwerveConstants.kPTheta, 0, 0);
    private CommandXboxController driveController = RobotContainer.getInstance().getDrive_Controller();
    PID turnPID = new PID(0.02,0.0,0.0);


    Optional<Pose2d> currentPoseOpt;
    int CurrentID;

    public AutoLookAtTarget(Supplier<Optional<Pose2d>> poseSupplier,  IntSupplier idSupplier, boolean left, DriveTrain swerve){
        this.poseSupplier = poseSupplier;
        this.idSupplier = idSupplier;
        this.left = left;
        this.swerve = swerve;

    }
    public AutoLookAtTarget( boolean bleft, DriveTrain swerve){
        left = bleft;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        
    }


    @Override
    public void execute(){
        currentPoseOpt = RobotContainer.getInstance().m_driveTrain.vision.lastCalculatedDist;  //should be converted to method
        timestamp = RobotContainer.getInstance().m_driveTrain.vision.getVisionTimestamp();
        CurrentID = RobotContainer.getInstance().m_driveTrain.vision.getLatestID();
    
    if(currentPoseOpt.isPresent() ) {

      Pose2d currentPose = currentPoseOpt.get();
      distFromTag = currentPose;

      //Pose2d currentPose = new Pose2d(2.0, 3.0, new Rotation2d(Math.PI / 4)); // x=2, y=3, 45 degrees
      Translation2d target = new Translation2d(5.0, 7.0);
      
      Rotation2d angleToTurn = getRelativeAngleToTarget(currentPose, target);
      //This it the distance to the target.
      double distance = getDistanceToTarget(currentPose, target);
      


      //double xPower = MathUtil.clamp(xController.calculate(currentPose.getX(), left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX), -1, 1);
      //double xPower =  50*CommonLogic.CapMotorPower(turnPID.calcPIDF(0, currentPose.getX()), -0.30, .3)  ;
      //double yPower = MathUtil.clamp(yController.calculate(currentPose.getY(), left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY), -1, 1);
      //double yPower =  50*CommonLogic.CapMotorPower(turnPID.calcPIDF(0, currentPose.getY()), -0.30, .3)  ;
      double drivePower = driveController.getLeftY();
      double strafePower = driveController.getLeftX();
      double thetaPower = thetaController.calculate(angleToTurn.getDegrees(), Constants.VisionConstants.thetaAlignment);
      //double thetaPower = thetaController.calculate(currentPose.getRotation().getDegrees(), Constants.VisionConstants.thetaAlignment);

      

      //double thetaPower =  5*CommonLogic.CapMotorPower(turnPID.calcPIDF(0, currentPose.getRotation().getDegrees()), -0.30, .3)  ;

      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentX",currentPose.getX());
      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentY",currentPose.getY());
      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentTheta",currentPose.getRotation().getRadians());
      SmartDashboard.putNumber("Subsystem/Vision/xPOut",drivePower);
      SmartDashboard.putNumber("Subsystem/Vision/yPOut",strafePower);
      SmartDashboard.putNumber("Subsystem/Vision/thetaOut",thetaPower);
       swerve.drive(new ChassisSpeeds(drivePower, strafePower, thetaPower));

       inTolarance();
    }
    

}
public Rotation2d getAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
    Translation2d toTarget = targetPosition.minus(currentPose.getTranslation());
    return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
}


public Rotation2d getRelativeAngleToTarget(Pose2d currentPose, Translation2d targetPosition) {
    Rotation2d angleToTarget = getAngleToTarget(currentPose, targetPosition);
    return angleToTarget.minus(currentPose.getRotation());
}

public double getDistanceToTarget(Pose2d currentPose, Translation2d targetPosition) {
    return currentPose.getTranslation().getDistance(targetPosition);
}



public void inTolarance(){
    if ((Math.abs(distFromTag.getX()-(left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX))<VisionConstants.xTolerance) 
    && (Math.abs(distFromTag.getY()-(left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY))<VisionConstants.yTolerance)
    && (Math.abs(distFromTag.getRotation().getRadians()-Constants.VisionConstants.thetaAlignment)<VisionConstants.thetaTolerance)){
        bdone = true;
    }
}


    @Override
    public boolean isFinished(){
        return bdone;
        
       
    } 
}
