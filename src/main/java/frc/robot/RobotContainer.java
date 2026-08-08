// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.MappingIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;

public class RobotContainer {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();
  private Supplier<Boolean> shouldLockPose = () -> mainController.a().getAsBoolean();
  private final IntakeSubsystem intakeSubsystem;
  public RobotContainer() {
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);
    intakeSubsystem = new IntakeSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    // swerveDrive.setDefaultCommand(new SwerveControlCmd(
    //     swerveDrive, mainController, shouldSprint, shouldLockPose));
    // mainController.start().onTrue(Commands.runOnce(() -> {
    //   swerveDrive.zeroGyro();
    //   swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    // }));
    mainController.a().whileTrue(intakeSubsystem.reverseIntakeCmd());
    mainController.b().whileTrue(intakeSubsystem.intakeCmd());
    mainController.povUp().whileTrue(intakeSubsystem.manualRetractPivotCmd());
    mainController.povDown().whileTrue(intakeSubsystem.manualDeployPivotCmd());
    mainController.y().onTrue(intakeSubsystem.autoDeployPivotCmd());
    mainController.a().onTrue(intakeSubsystem.autoRetractPivotCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}