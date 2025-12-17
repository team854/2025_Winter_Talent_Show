package frc.robot.Commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class OutakeCommand extends Command {
    private double startTime = 0;
    
    public OutakeCommand() {
        addRequirements(RobotContainer.indexerSubsystem);
    }

    @Override
    public void initialize() {
        startTime = 0;
        RobotContainer.indexerSubsystem.setPower(-1);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (!RobotContainer.indexerSubsystem.getProximity()) {
            if (startTime == 0) {
                startTime = Timer.getFPGATimestamp();
            }
            if ((Timer.getFPGATimestamp() - startTime) >= 1) {
                return true;
            }
        } else {
            startTime = 0;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.indexerSubsystem.setPower(0);
    }
}
