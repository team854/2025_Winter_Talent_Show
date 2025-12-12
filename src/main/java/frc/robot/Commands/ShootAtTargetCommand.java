package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degree;

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

        long startTime = System.nanoTime();

        TargetSolution targetSolution = new TargetSolution(TargetErrorCode.NONE, Degree.of(0), Degree.of(0));
        targetSolution = RobotContainer.projectileSubsystem.calculateLaunchAngleSimulation(
        Constants.TargetConstants.SHOOTER_VELOCITY,
        new Translation2d(),
        Constants.TargetConstants.TARGET_POSITION,
        Constants.TargetConstants.MAX_STEPS,
        Constants.TargetConstants.TPS
        );
        long endTime = System.nanoTime();
        // --- BENCHMARK END ---

        // Calculate duration in microseconds (1 ms = 1000 µs)
        long durationUs = ((endTime - startTime) / 1000) / 1;
        double durationMs = durationUs / 1000.0;

        System.out.println(String.format("Trajectory Calc Time: %d µs (%.3f ms)", durationUs, durationMs));

        if (TargetErrorCode.NONE != targetSolution.errorCode()) {
            System.out.println("Error target soluation failed: " + targetSolution.errorCode().toString());
            return;
        }

        System.out.println("TARGET PITCH:" + targetSolution.launchPitch().in(Degree));
        
        this.commands.addCommands(
            new ParallelCommandGroup(
                new SetArmAngleCommand(targetSolution.launchPitch()),
                new SetShooterSpeedCommand(Constants.ShooterConstants.SHOOTER_ANGULAR_VELOCITY)
            ),
            new ParallelCommandGroup(
                new OutakeCommand()
            )
        );
        
        this.commands.initialize();
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