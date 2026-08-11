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
import frc.robot.lib.shooting.ShotTable.CanShootInSpeed;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class AutoShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final SwerveDrive swerveDrive;
  private final ShotTable shotTable;

  private double targetVelocity;
  private double targetAngle;

  /**
   * 全場自動射球:
   * | Trench (含緩衝) -> Shooter/Feeder/Transport 全部強制停止，角度歸零，方便過Trench/Bump 
   * | Alliance zone -> 依距離查ShotTable瞄準Hub，轉速到位才餵球 
   * | Neutral ZONE -> 固定平飛角度+固定傳球轉速，轉速到位才餵球 |
   */
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
    boolean hasValidTarget = true;

    if (inTrench) {
      targetAngle = AngleConstants.angleMotorMinAngle;
      shooterSubsystem.stopShooter();
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
      angleSubsystem.angleSync(targetAngle);

      SmartDashboard.putString("autoShooter/autoZone", "trench");
      return;
    }

    if (inOwnZone) {
      Distance dis = Meters.of(robotPos.getDistance(FieldConstants.getHubPosition()));
      CanShootInSpeed solution = shotTable.pickClosestVelocity(
          dis.in(Centimeters), shooterSubsystem.getShooterVelocity());

      if (solution != null) {
        targetAngle = solution.angleDeg();
        targetVelocity = solution.velocityRpm();
      } else {
        hasValidTarget = false;
      }
      SmartDashboard.putNumber("autoShooterDistance", dis.in(Centimeters));
      zoneLabel = "own";

    } else {
      targetAngle = AngleConstants.angleMotorTransAngle;
      targetVelocity = ShooterConstants.passVelocity;
      zoneLabel = "middle";
    }

    shooterSubsystem.shoot(targetVelocity);
    angleSubsystem.angleSync(targetAngle);

    if (hasValidTarget && shooterSubsystem.isShooterAtSpeed()) {
      feederSubsystem.feedIn();
      transportSubsystem.transportIn();
    } else {
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
    }

    SmartDashboard.putString("autoShooter/autoZone", zoneLabel);
    SmartDashboard.putNumber("autoShooter/autoAngle", targetAngle);
    SmartDashboard.putNumber("autoShooter/autoVelocity", targetVelocity);
    SmartDashboard.putBoolean("autoShooter/atSpeed", shooterSubsystem.isShooterAtSpeed());
    SmartDashboard.putBoolean("autoShooter/hasValidTarget", hasValidTarget);
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