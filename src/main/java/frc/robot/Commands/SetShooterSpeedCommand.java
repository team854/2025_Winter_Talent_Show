package frc.robot.Commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetShooterSpeedCommand extends Command {
    private AngularVelocity targetSpeed;
    private double startTime = 0;

    public SetShooterSpeedCommand(AngularVelocity targetSpeed) {
        this.targetSpeed = targetSpeed;
        addRequirements(RobotContainer.shooterSubsystem);
    }
 
    @Override
    public void initialize() {
        startTime = 0;
        RobotContainer.shooterSubsystem.setSpeed(targetSpeed);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        double averageSpeed = (RobotContainer.shooterSubsystem.getMotor1CurrentSpeed().in(RPM) + RobotContainer.shooterSubsystem.getMotor2CurrentSpeed().in(RPM)) / 2.0;
        double shooterDifference = Math.abs(targetSpeed.in(RPM)) - Math.abs(averageSpeed);
        boolean inRange = Math.abs(shooterDifference) < (Constants.ShooterConstants.SHOOTER_ALLOWED_ERROR.in(RPM) * 50);

        if (inRange) {
            if (startTime == 0) {
                startTime = Timer.getFPGATimestamp();
            }
            if ((Timer.getFPGATimestamp() - startTime) >= 0.2) {
                return true;
            }
        } else {
            startTime = 0;
        }
        return false;
    }
}
