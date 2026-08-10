// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.shooting.ShotTable;
import frc.robot.lib.shooting.ShotTable.Candidate;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * 全場自動射球:
 * - trench 區(含緩衝) -> shooter/feeder/transport 全部強制停止，角度歸零，方便過trench/bump
 * - 本方區 -> 依距離查ShotTable瞄準hub，轉速到位才餵球
 * - 中場 -> 固定平飛角度+固定傳球轉速，轉速到位才餵球
 */
public class AutoShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final SwerveDrive swerveDrive;
  private final ShotTable shotTable;

  private double targetVelocity;
  private double targetAngle;

  public AutoShootCmd(ShooterSubsystem shooterSubsystem,
      AngleSubsystem angleSubsystem,
      FeederSubsystem feederSubsystem,
      TransportSubsystem transportSubsystem,
      SwerveDrive swerveDrive,
      ShotTable shotTable) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.transportSubsystem = transportSubsystem;
    this.swerveDrive = swerveDrive;
    this.shotTable = shotTable;
    addRequirements(shooterSubsystem, angleSubsystem, feederSubsystem, transportSubsystem);
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);
    boolean inOwnZone = isInOwnZone(robotPos);

    String zoneLabel;

    if (inTrench) {
      // trench 優先權最高: shooter 必須完全停止，角度歸零方便過障礙
      targetAngle = AngleConstants.angleMotorMinAngle;
      shooterSubsystem.stopShooter();
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
      angleSubsystem.angleSync(targetAngle);

      SmartDashboard.putString("shooter/autoZone", "trench");
      return; // trench 時提早結束，不執行下面的餵球判斷
    }

    if (inOwnZone) {
      Distance dis = Meters.of(robotPos.getDistance(FieldConstants.getHubPosition()));
      Candidate solution = shotTable.pickClosestVelocity(
          dis.in(Centimeters), shooterSubsystem.getShooterVelocity());

      if (solution != null) {
        targetAngle = solution.angleDeg();
        targetVelocity = solution.velocityRpm();
      }
      SmartDashboard.putNumber("shooterDistance", dis.in(Centimeters));
      zoneLabel = "own";

    } else {
      targetAngle = AngleConstants.angleMotorTransAngle;
      targetVelocity = ShooterConstants.passVelocity;
      zoneLabel = "middle";
    }

    shooterSubsystem.shoot(targetVelocity);
    angleSubsystem.angleSync(targetAngle);

    if (shooterSubsystem.isShooterAtSpeed()) {
      feederSubsystem.feedIn();
      transportSubsystem.transportIn();
    } else {
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
    }

    SmartDashboard.putString("shooter/autoZone", zoneLabel);
    SmartDashboard.putNumber("shooter/autoAngle", targetAngle);
    SmartDashboard.putNumber("shooter/autoVelocity", targetVelocity);
    SmartDashboard.putBoolean("shooter/atSpeed", shooterSubsystem.isShooterAtSpeed());
  }

  private boolean isInOwnZone(Translation2d robotPos) {
    double x = robotPos.getX();
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return x >= FieldZones.redTrenchZoneMaxX;
    }
    return x <= FieldZones.blueTrenchZoneMinX;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    feederSubsystem.feedStop();
    transportSubsystem.stopTransport();
    angleSubsystem.lockCurrentAngle();
  }
}