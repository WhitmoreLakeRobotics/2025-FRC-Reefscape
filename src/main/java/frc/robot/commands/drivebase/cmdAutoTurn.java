package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class cmdAutoTurn extends Command {
    private boolean bDone = false;
    private double bHeading;
    private double rHeading;

    
    public cmdAutoTurn(double blueHeading, double redHeading) {

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        bHeading =  blueHeading;
        rHeading = redHeading;

    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == Alliance.Red){
        RobotContainer.getInstance().m_driveTrain.setTurnHeading(rHeading);
        RobotContainer.getInstance().m_driveTrain.setAdutoTurn();
    } else {
        RobotContainer.getInstance().m_driveTrain.setTurnHeading(bHeading);
        RobotContainer.getInstance().m_driveTrain.setAdutoTurn();

    }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        bDone = true;
        end(false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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