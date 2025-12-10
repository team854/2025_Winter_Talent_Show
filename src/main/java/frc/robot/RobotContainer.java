// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ActivateShooterCommand;
import frc.robot.Commands.DefaultArmCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.ShootAtTargetCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ProjectileSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {
  public static final CommandXboxController xboxController = new CommandXboxController(0);
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static final ProjectileSubsystem projectileSubsystem = new ProjectileSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new DefaultDriveCommand());
    armSubsystem.setDefaultCommand(new DefaultArmCommand());

    xboxController.a().whileTrue(new ActivateShooterCommand());

    //xboxController.b().onTrue(new ShootAtTargetCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
