package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurnRobotAngleCommand extends Command {
    private Angle targetAngle;
    private double startTime = 0;
    private final PIDController pid = new PIDController(
                        Constants.DriveConstants.DRIVE_PID_P,
                        Constants.DriveConstants.DRIVE_PID_I, 
                        Constants.DriveConstants.DRIVE_PID_D);

    public TurnRobotAngleCommand(Angle targetAngle) {
        pid.enableContinuousInput(0, 360);

        this.targetAngle = targetAngle;
        addRequirements(RobotContainer.driveSubsystem);
    }
 
    @Override
    public void initialize() {
        startTime = 0;
        pid.setTolerance(Constants.DriveConstants.DRIVE_ALLOWED_ERROR.in(Degree));
        pid.setSetpoint(this.targetAngle.in(Degree));
    }

    @Override
    public void execute() {
        double pidOutput = MathUtil.clamp(pid.calculate(RobotContainer.driveSubsystem.getGyroFacing().in(Degree)), -0.15, 0.15);

        RobotContainer.driveSubsystem.setDriveMotors(pidOutput * 0.9, pidOutput * -1.1);
    }

    @Override
    public void end(boolean isFinished) {
        RobotContainer.driveSubsystem.setDriveMotors(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        if (pid.atSetpoint()) {
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
