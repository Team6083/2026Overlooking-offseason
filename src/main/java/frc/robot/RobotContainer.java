// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.databind.MappingIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.AdjustSpeedAngleCmd;
import frc.robot.commands.AutoAngleCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.AngleSubsystem.AnglePreset;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;
import java.util.function.Supplier;

public class RobotContainer {
  private final CommandXboxController copilotController = new CommandXboxController(1);
  private final CommandXboxController mainController = new CommandXboxController(0);
  
  private SwerveDrive swerveDrive;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final Supplier<Boolean> shouldSprint = () -> mainController.rightTrigger().getAsBoolean();
  private final Supplier<Boolean> shouldLockPose = () -> mainController.a().getAsBoolean();

  
  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);

    angleSubsystem.setDistanceSupplier(() -> Meters.of(swerveDrive.getPose2d().getTranslation()
        .getDistance(FieldConstants.getHubPosition()))
        .in(Centimeters));
    angleSubsystem.angleSyncCmd(20).schedule();
    //angleSubsystem.setDefaultCommand(new AutoAngleCmd(angleSubsystem, swerveDrive));
    intakeSubsystem = new IntakeSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController, shouldSprint, shouldLockPose));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));

    // shooterSubsystem.setDefaultCommand(shooterSubsystem.shootCmd(ShooterConstants.shooterLowGearTarget));
    mainController.povUp().onTrue(new AutoAngleCmd(angleSubsystem, swerveDrive));
    mainController.leftTrigger().whileTrue(
        new AdjustSpeedAngleCmd(shooterSubsystem,angleSubsystem, swerveDrive));
            //.alongWith(aimAtHubCmd()));

    copilotController.povUp().onTrue(angleSubsystem.angleSyncCmd(20));
    copilotController.povDown().onTrue(angleSubsystem.angleSyncCmd(30));
    copilotController.povLeft().onTrue(angleSubsystem.angleSyncCmd(40));
    copilotController.povRight().onTrue(angleSubsystem.angleSyncCmd(50));
    mainController.a().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.TRANS));
    mainController.b().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.MAX));
    mainController.x().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE));
    mainController.y().onTrue(Commands.runOnce(angleSubsystem::lockCurrentAngle, angleSubsystem));
    mainController.leftBumper().whileTrue(shooterSubsystem.shootCmd());

    copilotController.rightBumper().whileTrue(feederSubsystem.feedInCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}