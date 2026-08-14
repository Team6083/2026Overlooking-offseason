// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSubsystem;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAssistCmd;
import frc.robot.commands.AutoAngleCmd;
import frc.robot.commands.ManualJoystickCmd;
import frc.robot.commands.ShooterComboCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.commands.manualShooterComboCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.lib.FieldUtil;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.AngleSubsystem.AnglePreset;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;
import frc.robot.subsystems.vision.VisionIoLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);

  private Supplier<Boolean> shouldSprint = () -> mainController.x().getAsBoolean();
  private Supplier<Boolean> shouldLockPose = () -> mainController.b().getAsBoolean();
  private final SendableChooser<Command> autoChooser;
  private SwerveDrive swerveDrive;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final VisionSubsystem visionSubsystem;


  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    angleSubsystem = new AngleSubsystem();
    transportSubsystem = new TransportSubsystem();
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);


    Auto.configureAutoBuilder(swerveDrive);

    registerCommand();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser", autoChooser);
    visionSubsystem = new VisionSubsystem(
        new VisionIoLimelight(
            // MegaTag2 要的是場地座標下的朝向。estimator 的 rotation 才是 gyro + offset
            // 之後的場地朝向;vision 不再修正旋轉,所以它等同於「gyro 加上開賽時對齊的偏移」。
            () -> swerveDrive.getPose2d().getRotation().getDegrees(),
            () -> Math.toDegrees(swerveDrive.getRobotRelativeSpeeds().omegaRadiansPerSecond),
            "limelight-intake", "limelight-shooter"),
        swerveDrive::getPose2d,
        (pose, timestamp, stdDevs) -> swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs));

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

    mainController.rightTrigger().whileTrue(intakeSubsystem.intakeCmd());
    mainController.rightBumper().whileTrue(intakeSubsystem.reverseIntakeCmd());
    mainController.leftBumper()
        .whileTrue(new manualShooterComboCmd(
            shooterSubsystem, feederSubsystem,
            transportSubsystem, angleSubsystem));
    mainController.leftTrigger().whileTrue(
        new ShooterComboCmd(
            swerveDrive, shooterSubsystem,
            transportSubsystem, feederSubsystem,
            intakeSubsystem, angleSubsystem,
            () -> true)
            .alongWith(new AimAssistCmd(
                swerveDrive, mainController,
                shouldSprint, shouldLockPose)));

    // 副 Driver
    copilotController.a().whileTrue(intakeSubsystem.manualDeployPivotCmd()); 
    copilotController.y().whileTrue(intakeSubsystem.manualRetractPivotCmd());
    copilotController.b().whileTrue(intakeSubsystem.retakePivotCmd());
    copilotController.x()
        .onTrue(angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE)
            .alongWith(shooterSubsystem.shootCmd(0)));

    copilotController.rightTrigger().whileTrue(transportSubsystem.transportInCmd());
    copilotController.leftTrigger().whileTrue(feederSubsystem.feedInCmd());

    copilotController.povUp().whileTrue(shooterSubsystem.shootCmd());
    copilotController.leftBumper().whileTrue(
        new ManualJoystickCmd(angleSubsystem, intakeSubsystem, copilotController));
  }

   private void registerCommand() {
    NamedCommands.registerCommand("Intake", intakeSubsystem.intakeCmd());
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intakeSubsystem.stopIntake()));
    NamedCommands.registerCommand("DeployIntake", intakeSubsystem.manualDeployPivotCmd().withTimeout(1.2));
    NamedCommands.registerCommand("RetractIntake", intakeSubsystem.manualRetractPivotCmd());
    NamedCommands.registerCommand("Shoot", shooterSubsystem.shootCmd().withTimeout(4));
    NamedCommands.registerCommand("adjustAngle" , angleSubsystem.adjustAngleCmd(AnglePreset.CLOSE));
    NamedCommands.registerCommand("shootAngle" , angleSubsystem.adjustAngleCmd(AnglePreset.MAX));

   }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}