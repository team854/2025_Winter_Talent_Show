// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ActivateShooterCommand;
import frc.robot.Commands.DefaultArmCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ShootAtTargetCommand;
import frc.robot.Commands.TurnRobotAngleCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.ProjectileSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {
  public static final CommandXboxController xboxController = new CommandXboxController(0);
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final ProjectileSubsystem projectileSubsystem = new ProjectileSubsystem();
  public static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  // public static final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand());
    armSubsystem.setDefaultCommand(new DefaultArmCommand());

    xboxController.a().onTrue(new IntakeCommand());

    xboxController.x().onTrue(
      new InstantCommand(() -> {
        driveSubsystem.zeroGyro();
      }));

    xboxController.y().onTrue(new ShootAtTargetCommand(
      new Translation3d(4, 0, 1.25)
    ));

    xboxController.b().onTrue(new ShootAtTargetCommand(
      new Translation3d(4, 1.9, 1.3)
    ));

    //xboxController.b().onTrue(new ShootAtTargetCommand());
  }

  public Command getAutonomousCommand() {
    return new TurnRobotAngleCommand(Degree.of(15));
    //return new ShootAtTargetCommand();
  }
}
