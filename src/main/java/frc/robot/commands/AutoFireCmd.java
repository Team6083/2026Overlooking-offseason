// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.shooting.ShotTable;
import frc.robot.lib.shooting.ShotTable.CanShootInSpeed;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * 主Driver: 依距離查ShotTable決定轉速並在到速時餵球(按著才動)。
 * Trench 中強制停止，避免過障礙時噴球。
 */
public class AutoFireCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final SwerveDrive swerveDrive;
  private final ShotTable shotTable;

  private double targetVelocity;

  public AutoFireCmd(ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      TransportSubsystem transportSubsystem,
      SwerveDrive swerveDrive,
      ShotTable shotTable) {
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.transportSubsystem = transportSubsystem;
    this.swerveDrive = swerveDrive;
    this.shotTable = shotTable;
    addRequirements(shooterSubsystem, feederSubsystem, transportSubsystem);
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);

    if (inTrench) {
      shooterSubsystem.stopShooter();
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
      SmartDashboard.putString("autoFire/zone", "Trench");
      return;
    }

    boolean inOwnZone = isInOwnZone(robotPos);
    boolean hasValidTarget = true;

    if (inOwnZone) {
      Distance dis = Meters.of(robotPos.getDistance(FieldConstants.getHubPosition()));
      CanShootInSpeed solution = shotTable.pickClosestVelocity(
          dis.in(Centimeters), shooterSubsystem.getShooterVelocity());

      if (solution != null) {
        targetVelocity = solution.velocityRpm();
      } else {
        hasValidTarget = false;
      }
      SmartDashboard.putNumber("autoFire/distance", dis.in(Centimeters));
    } else {
      targetVelocity = ShooterConstants.passVelocity;
    }

    shooterSubsystem.shoot(targetVelocity);

    if (hasValidTarget && shooterSubsystem.isShooterAtSpeed()) {
      feederSubsystem.feedIn();
      transportSubsystem.transportIn();
    } else {
      feederSubsystem.feedStop();
      transportSubsystem.stopTransport();
    }

    SmartDashboard.putString("autoFire/zone", inOwnZone ? "Alliance zone" : "Neutral zone");
    SmartDashboard.putNumber("autoFire/velocity", targetVelocity);
    SmartDashboard.putBoolean("autoFire/atSpeed", shooterSubsystem.isShooterAtSpeed());
    SmartDashboard.putBoolean("autoFire/hasValidTarget", hasValidTarget);
  }

  private boolean isInOwnZone(Translation2d robotPos) {
    double x = robotPos.getX();
    if (edu.wpi.first.wpilibj.DriverStation.getAlliance().isPresent()
        && edu.wpi.first.wpilibj.DriverStation.getAlliance().get()
            == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
      return x >= FieldZones.redTrenchZoneMaxX;
    }
    return x <= FieldZones.blueTrenchZoneMinX;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    feederSubsystem.feedStop();
    transportSubsystem.stopTransport();
  }
}