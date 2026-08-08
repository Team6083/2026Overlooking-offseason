// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AngleSubsystem.AnglePreset;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;

public class RobotContainer {
  private final SwerveDrive swerveDrive;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final CommandXboxController mainController = new CommandXboxController(0);
  private Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();
  private Supplier<Boolean> shouldLockPose = () -> mainController.a().getAsBoolean();

  public RobotContainer() {
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);
    configureBindings();

  }

  private void configureBindings() {
    // swerveDrive.setDefaultCommand(new SwerveControlCmd(
    // swerveDrive, mainController, shouldSprint, shouldLockPose));
    // mainController.start().onTrue(Commands.runOnce(() -> {
    // swerveDrive.zeroGyro();
    // swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(),
    // Rotation2d.fromDegrees(0)));

    // }));
    mainController.a().whileTrue(shooterSubsystem.shootCmd());
    mainController.b().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE));
    mainController.x().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.SHOOT));
    mainController.y().onTrue(Commands.runOnce(angleSubsystem::lockCurrentAngle, angleSubsystem));
}
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}