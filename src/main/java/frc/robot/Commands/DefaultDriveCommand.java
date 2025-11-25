package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DefaultDriveCommand extends Command {
    public DefaultDriveCommand() {
        addRequirements(RobotContainer.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double foward = -RobotContainer.xboxController.getLeftY() * Constants.ControlConstants.FORWARD_SPEED;
        double turn = -RobotContainer.xboxController.getLeftX() * Constants.ControlConstants.RIGHT_SPEED;

        double leftMotor = foward - turn;
        double rightMotor = foward + turn;
        RobotContainer.driveSubsystem.setDriveMotors(leftMotor,rightMotor);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveSubsystem.setDriveMotors(0,0);
    }
}
