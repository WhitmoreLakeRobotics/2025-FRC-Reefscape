package frc.robot.commands.drivebase;

import javax.swing.RootPaneContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;

public class DriveToPosCancel extends Command {
    private boolean bDone = false;
    private double bHeading;
    private double rHeading;
    private int latestID;
    private Pose2d newTarget;

    
    public DriveToPosCancel() {

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        

    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
        latestID = RobotContainer.getInstance().m_driveTrain.vision.getLatestID();
        newTarget = RobotContainer.getInstance().m_driveTrain.returnPose2d(latestID);

   
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        CommandScheduler.getInstance().cancel(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget)); 
        bDone = true;
        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //CommandScheduler.getInstance().cancel(RobotContainer.getInstance().m_driveTrain.driveToPose(newTarget));
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
}