package frc.robot.Commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmAngleCommand extends Command {
    private final Angle targetArmAngle;
    public SetArmAngleCommand(Angle targetArmAngle) {
        this.targetArmAngle = targetArmAngle;
        addRequirements(RobotContainer.armSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.armSubsystem.setTargetArmAngle(this.targetArmAngle);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        double angleDifference = targetArmAngle.in(Radians) - RobotContainer.armSubsystem.getCurrentArmAngle().in(Radians);
        return Math.abs(angleDifference) < Constants.ArmConstants.ARM_ALLOWED_ERROR.in(Radians);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
