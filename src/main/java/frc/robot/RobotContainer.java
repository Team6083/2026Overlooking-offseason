// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.AngleSubsystem.AnglePreset;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;
import java.util.function.Supplier;

public class RobotContainer {
  private final CommandXboxController mainController = new CommandXboxController(0);
  private SwerveDrive swerveDrive;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();
  private final Supplier<Boolean> shouldLockPose = () -> mainController.a().getAsBoolean();

  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
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
    mainController.a().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.TRANS));
    mainController.b().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.MAX));
    mainController.x().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE));
    mainController.y().onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.SHOOT));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}