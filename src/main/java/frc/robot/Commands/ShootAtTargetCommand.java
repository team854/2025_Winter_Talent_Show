package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ProjectileSubsystem;
import frc.robot.Subsystems.ProjectileSubsystem.TargetErrorCode;
import frc.robot.Subsystems.ProjectileSubsystem.TargetSolution;

public class ShootAtTargetCommand extends Command {

    private SequentialCommandGroup commands;
    public ShootAtTargetCommand() {
        
    }

    @Override
    public void initialize() {
        this.commands = new SequentialCommandGroup();

        TargetSolution targetSolution = RobotContainer.projectileSubsystem.calculateLaunchAngleSimulation(
            Constants.TargetConstants.SHOOTER_VELOCITY,
            new Translation2d(),
            Constants.TargetConstants.TARGET_POSITION,
            Constants.TargetConstants.MAX_STEPS,
            Constants.TargetConstants.TPS
        );

        if (TargetErrorCode.NONE != targetSolution.errorCode()) {
            System.out.println("Error target soluation failed: " + targetSolution.errorCode().toString());
            return;
        }
        this.commands.addCommands(
            new ParallelCommandGroup(
                new SetArmAngleCommand(targetSolution.launchPitch()),
                new SetShooterSpeedCommand(Constants.ShooterConstants.SHOOTER_MAX_ANGULAR_VELOCITY)
            )
        );
    }

    @Override
    public void execute() {
        this.commands.execute();
    }

    @Override
    public boolean isFinished() {
        return this.commands.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        this.commands.end(interrupted);
    }
}