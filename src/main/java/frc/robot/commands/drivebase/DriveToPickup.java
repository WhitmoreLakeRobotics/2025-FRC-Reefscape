package frc.robot.commands.drivebase;

import javax.swing.RootPaneContainer;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.utils.CommonLogic;

public class DriveToPickup extends Command {
    private boolean bDone = false;
    private double bHeading;
    private double rHeading;
    private int latestID;
    private Pose2d newTarget;
    private boolean isLeft = true;
    private int isBlue = -1;

    
    public DriveToPickup(boolean isLeft2) {
        this.isLeft = isLeft2;
        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        

    }

    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
       // latestID = RobotContainer.getInstance().m_driveTrain.vision.getLatestID();
        newTarget = returnPosePickup( isLeft, CommonLogic.getIsBlue());

   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        CommandScheduler.getInstance().schedule(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget)); 
        bDone = true;
        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().cancel(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget));
        bDone = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return bDone;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
  public Pose2d returnPosePickup(boolean isLeft,int isBlue) {
    if(isLeft){
      if(isBlue == 1){
        //is left and blue
        return TARGETPOS.BPICKUP.leftPose2d;
      }else {  //is red  
        //is left and red
        return TARGETPOS.PICKUP.leftPose2d;
      }
    }else{  //is right
      if(isBlue == 1){
        //is right and blue
        return TARGETPOS.BPICKUP.rightPose2d;
      }else {  //is red
        //is right and red
        return TARGETPOS.PICKUP.rightPose2d;
        
      }
    }


  }

  public enum TARGETPOS {

    PICKUP(new Pose2d(new Translation2d(15.915, 0.620), Rotation2d.fromDegrees(-60.00)),
          new Pose2d(new Translation2d(15.951, 7.400),Rotation2d.fromDegrees(60.00))),
    BPICKUP(new Pose2d(new Translation2d(1.685, 7.380), Rotation2d.fromDegrees(120.00)),
            new Pose2d(new Translation2d(1.604, 0.693),Rotation2d.fromDegrees(-120.00))),
    DEFAULT(new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0)),
              new Pose2d(new Translation2d(16.0, 4.5), Rotation2d.fromDegrees(0)));
      





    private Pose2d leftPose2d;
    private Pose2d rightPose2d;


    TARGETPOS(Pose2d leftPose2d, Pose2d rightPose2d){
      this.leftPose2d = leftPose2d;
      this.rightPose2d = rightPose2d;
    }


    public Pose2d getLefPose2d(){
      return leftPose2d;

    }
    public Pose2d getRightPose2d(){
      return rightPose2d;

    }
  }
}