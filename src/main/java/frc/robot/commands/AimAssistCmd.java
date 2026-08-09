// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AimAssistConstant;
import frc.robot.Constants.FieldConstant;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAssistCmd extends SwerveControlCmd {

  private final PIDController yawPid;
  private final Debouncer alignedDebouncer =
      new Debouncer(AimAssistConstant.kAlignDebounceSec, Debouncer.DebounceType.kRising);
  private final BooleanSupplier poseTrusted;
  private boolean aligned;

  public AimAssistCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint, Supplier<Boolean> shouldLock) {
    this(swerveDrive, mainController, shouldSprint, shouldLock, null);
  }

  public AimAssistCmd(SwerveDrive swerveDrive, CommandXboxController mainController,
      Supplier<Boolean> shouldSprint, Supplier<Boolean> shouldLock, BooleanSupplier poseTrusted) {
    super(swerveDrive, mainController, shouldSprint, shouldLock);
    this.poseTrusted = poseTrusted;

    yawPid = new PIDController(AimAssistConstant.kP, AimAssistConstant.kI, AimAssistConstant.kD);
    yawPid.enableContinuousInput(-180, 180);
    yawPid.setIZone(AimAssistConstant.kIZoneDeg);
    yawPid.setTolerance(AimAssistConstant.kToleranceDeg);
  }

  @Override
  public void initialize() {
    super.initialize();
    yawPid.reset();
    aligned = false;
  }

  @Override
  protected double calcRotSpeed() {
    if (Math.abs(mainController.getRightX()) > AimAssistConstant.kDriverRotDeadband) {
      aligned = false;
      return super.calcRotSpeed();
    }
    if (poseTrusted != null && !poseTrusted.getAsBoolean()) {
      aligned = false;
      return super.calcRotSpeed();
    }

    Pose2d pose = swerveDrive.getPose2d();
    double currentDeg = pose.getRotation().getDegrees();
    double targetDeg = desiredHeading(pose).getDegrees();

    double output = MathUtil.clamp(
        yawPid.calculate(currentDeg, targetDeg),
        -AimAssistConstant.kMaxRotOutput, AimAssistConstant.kMaxRotOutput);

    aligned = alignedDebouncer.calculate(yawPid.atSetpoint());
    double actualOutput = aligned ? 0.0 : output;

    SmartDashboard.putNumber("AimAssist/targetDeg", targetDeg);
    SmartDashboard.putNumber("AimAssist/currentDeg", currentDeg);
    SmartDashboard.putNumber("AimAssist/output", actualOutput);
    SmartDashboard.putBoolean("AimAssist/aligned", aligned);

    return actualOutput;
  }

  private Rotation2d desiredHeading(Pose2d pose) {
    Translation2d toHub = hubPosition().minus(pose.getTranslation());
    Rotation2d heading = toHub.getAngle().plus(AimAssistConstant.kShooterYawOffset);
    if (AimAssistConstant.kBallSpeedMps > 0.0) {
      heading = heading.plus(motionLead(pose, toHub));
    }
    return heading;
  }

  private Rotation2d motionLead(Pose2d pose, Translation2d toHub) {
    ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(
        swerveDrive.getRobotRelativeSpeeds(), pose.getRotation());

    double norm = toHub.getNorm();
    if (norm < 1e-6) {
      return Rotation2d.kZero;
    }
    Translation2d dir = toHub.div(norm);

    double vAlong =
        fieldRel.vxMetersPerSecond * dir.getX() + fieldRel.vyMetersPerSecond * dir.getY();
    double vPerp =
        -fieldRel.vxMetersPerSecond * dir.getY() + fieldRel.vyMetersPerSecond * dir.getX();

    double effectiveBallSpeed = AimAssistConstant.kBallSpeedMps + vAlong;
    if (effectiveBallSpeed <= 1e-6) {
      return Rotation2d.kZero;
    }
    return new Rotation2d(Math.atan2(-vPerp, effectiveBallSpeed));
  }

  private Translation2d hubPosition() {
    boolean isRed = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    return isRed ? FieldConstant.kRedHub : FieldConstant.kBlueHub;
  }

  public boolean isAlignedToHub() {
    return aligned;
  }
}
