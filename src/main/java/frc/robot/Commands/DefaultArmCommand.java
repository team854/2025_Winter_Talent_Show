package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DefaultArmCommand extends Command {

    public DefaultArmCommand() {
        addRequirements(RobotContainer.armSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double currentArmAngle = RobotContainer.armSubsystem.getTargetArmAngle().in(Degrees);

        double armSpeed = -RobotContainer.xboxController.getRightY() * Constants.ControlConstants.ARM_SPEED;

        if (Math.abs(armSpeed) < 0.1) {
            armSpeed = 0;
        } else {
            RobotContainer.armSubsystem.setTargetArmAngle(Degrees.of(currentArmAngle + armSpeed));
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
