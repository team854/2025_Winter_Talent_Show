package frc.robot.Commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ActivateShooterCommand extends Command {
    public ActivateShooterCommand() {
        addRequirements(RobotContainer.shooterSubsystem);
    }
 
    @Override
    public void initialize() {
        System.out.println("STARTING");
        RobotContainer.shooterSubsystem.setSpeed(Constants.ShooterConstants.SHOOTER_ANGULAR_VELOCITY);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("EXITING");
        RobotContainer.shooterSubsystem.setSpeed(RPM.of(0));
    }
}
