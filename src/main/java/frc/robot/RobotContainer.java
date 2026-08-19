// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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

  private Supplier<Boolean> shouldSprint = () -> mainController.x().getAsBoolean();
  private Supplier<Boolean> shouldLockPose = () -> mainController.b().getAsBoolean();
  private SwerveDrive swerveDrive;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final Field2d field = new Field2d();

  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    transportSubsystem = new TransportSubsystem();
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);
    visionSubsystem = new VisionSubsystem(
        new VisionIoLimelight(
            // MegaTag2 要的是場地座標下的朝向。estimator 的 rotation 才是 gyro + offset
            // 之後的場地朝向;vision 不再修正旋轉,所以它等同於「gyro 加上開賽時對齊的偏移」。
            () -> swerveDrive.getPose2d().getRotation().getDegrees(),
            () -> Math.toDegrees(swerveDrive.getRobotRelativeSpeeds().omegaRadiansPerSecond),
            "limelight-intake", "limelight-shooter"),
        swerveDrive::getPose2d,
        (pose, timestamp, stdDevs) -> swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs));

    SmartDashboard.putData("Field", field); // 提早註冊一次
    configureBindings();
  }

  public void putRobotPoseOnDashboard() {
    field.setRobotPose(swerveDrive.getPose2d());
    SmartDashboard.putData("Robot Pose", field);
  }

  private void configureBindings() {
    // swerve
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController, shouldSprint, shouldLockPose));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}