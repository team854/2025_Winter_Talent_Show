package frc.robot.Commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetShooterSpeedCommand extends Command {
    private AngularVelocity targetSpeed;
    public SetShooterSpeedCommand(AngularVelocity targetSpeed) {
        this.targetSpeed = targetSpeed;
        addRequirements(RobotContainer.shooterSubsystem);
    }
 
    @Override
    public void initialize() {
        RobotContainer.shooterSubsystem.setSpeed(targetSpeed);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        double averageSpeed = (RobotContainer.shooterSubsystem.getMotor1CurrentSpeed().in(RPM) + RobotContainer.shooterSubsystem.getMotor2CurrentSpeed().in(RPM)) / 2.0;
        double shooterDifference = targetSpeed.in(RPM) - averageSpeed;
        return Math.abs(shooterDifference) < (Constants.ShooterConstants.SHOOTER_ALLOWED_ERROR.in(RPM) * 2);
    }
}
