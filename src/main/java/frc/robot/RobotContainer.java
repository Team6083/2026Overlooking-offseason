// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAssistCmd;
import frc.robot.commands.AutoAngleCmd;
import frc.robot.commands.ManualAngleJoystickCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.lib.FieldUtil;
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
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);

  private SwerveDrive swerveDrive;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final Supplier<Boolean> shouldSprint = () -> mainController.x().getAsBoolean();
  private final Supplier<Boolean> shouldLockPose = () -> mainController.b().getAsBoolean();

  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    angleSubsystem = new AngleSubsystem();
    transportSubsystem = new TransportSubsystem();
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);

    angleSubsystem.setDistanceSupplier(() -> Meters.of(swerveDrive.getPose2d().getTranslation()
        .getDistance(FieldUtil.getHubPosition()))
        .in(Centimeters));
    angleSubsystem.angleSyncCmd(15).schedule();

    configureBindings();
  }

  private void configureBindings() {
    // 主 Driver
    // swerve
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController, shouldSprint, shouldLockPose));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));

    angleSubsystem.setDefaultCommand(new AutoAngleCmd(angleSubsystem,
        swerveDrive));

    mainController.rightTrigger().onTrue(intakeSubsystem.intakeCmd());
    mainController.rightBumper().onTrue(intakeSubsystem.reverseIntakeCmd());
    mainController.leftTrigger().whileTrue(
        new ShooterComboCmd(
            swerveDrive, shooterSubsystem,
            transportSubsystem, feederSubsystem,
            intakeSubsystem, angleSubsystem,
            () -> true)
            .alongWith(new AimAssistCmd(swerveDrive, mainController, shouldSprint, shouldLockPose)));

    // 副 Driver
    copilotController.a().whileTrue(intakeSubsystem.deployPivotCmd());
    copilotController.y().whileTrue(intakeSubsystem.retractPivotCmd());
    copilotController.b().whileTrue(intakeSubsystem.retakePivotCmd());
    copilotController.x()
        .onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE)
            .alongWith(shooterSubsystem.shootCmd(0)));

    copilotController.rightTrigger().whileTrue(transportSubsystem.transportInCmd());
    copilotController.leftTrigger().whileTrue(feederSubsystem.feedInCmd());

    copilotController.povUp().whileTrue(shooterSubsystem.shootCmd());
    copilotController.leftBumper().whileTrue(
        new ManualAngleJoystickCmd(angleSubsystem, copilotController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}