package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

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
            this.commands.addCommands();
        } else {

            System.out.println("TARGET PITCH:" + targetSolution.launchPitch().in(Degree));
            System.out.println("TARGET Yaw:" + targetSolution.launchYaw().in(Degree));
            
            this.commands.addCommands(
                new ParallelCommandGroup(
                    new SetArmAngleCommand(targetSolution.launchPitch()),
                    new SetShooterSpeedCommand(Constants.ShooterConstants.SHOOTER_ANGULAR_VELOCITY),
                    new TurnRobotAngleCommand(targetSolution.launchYaw())
                ).withTimeout(Millisecond.of(4000)),
                new ParallelCommandGroup(
                    new OutakeCommand()
                ).withTimeout(Millisecond.of(3000)),
                new ParallelCommandGroup(
                    new SetShooterSpeedCommand(RPM.of(0.0))
                ).withTimeout(Millisecond.of(1000))
            );
        }

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