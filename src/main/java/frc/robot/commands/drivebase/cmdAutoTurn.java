package frc.robot.commands.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class cmdAutoTurn extends Command {
    private boolean bDone = false;
    private double tHeading;

    
    public cmdAutoTurn(double nHeading) {

        // m_subsystem = subsystem;
        // addRequirements(m_subsystem);

        tHeading = nHeading;

    }
    // if fixedDist = false => stagPosition is suposed to recieve the percantage to
    // be traversed in stag, in 0.xx format

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        bDone = false;
        RobotContainer.getInstance().m_driveTrain.setTurnHeading(tHeading);
        RobotContainer.getInstance().m_driveTrain.setAdutoTurn();


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