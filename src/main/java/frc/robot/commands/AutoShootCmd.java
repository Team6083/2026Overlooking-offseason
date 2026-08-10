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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * 依全場位置自動決定角度與轉速:
 * - trench 區(含緩衝) -> 角度歸零、轉速維持不變，方便過trench/bump
 * - 本方區(trench之前) -> 依距離查ShotTable，精準瞄準hub命中角度+轉速
 * - 中場(過了自己trench之後) -> 固定平飛角度(TRANS) + 固定傳球轉速
 */
public class AutoShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final SwerveDrive swerveDrive;
  private final ShotTable shotTable;

  private double targetVelocity;
  private double targetAngle;

  public AutoShootCmd(ShooterSubsystem shooterSubsystem,
      AngleSubsystem angleSubsystem,
      SwerveDrive swerveDrive,
      ShotTable shotTable) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.swerveDrive = swerveDrive;
    this.shotTable = shotTable;
    addRequirements(shooterSubsystem, angleSubsystem);
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);
    boolean inOwnZone = isInOwnZone(robotPos);

    String zoneLabel;

    if (inTrench) {
      // trench 優先權最高，角度歸零方便過障礙，轉速維持不變(不要在顛簸期間亂調)
      targetAngle = AngleConstants.angleMotorMinAngle;
      zoneLabel = "trench";

    } else if (inOwnZone) {
      // 本方區：依距離查表，精準瞄準hub
      Distance dis = Meters.of(robotPos.getDistance(FieldConstants.getHubPosition()));
      Candidate solution = shotTable.pickClosestVelocity(
          dis.in(Centimeters), shooterSubsystem.getShooterVelocity());

      if (solution != null) {
        targetAngle = solution.angleDeg();
        targetVelocity = solution.velocityRpm();
      }
      // 若超出查表範圍(solution為null)，維持上一輪的值，不要亂噴
      SmartDashboard.putNumber("shooterDistance", dis.in(Centimeters));
      zoneLabel = "own";

    } else {
      // 中場：固定平飛角度傳球
      targetAngle = AngleConstants.angleMotorTransAngle;
      targetVelocity = ShooterConstants.passVelocity;
      zoneLabel = "middle";
    }

    shooterSubsystem.shootCmd(targetVelocity);
    angleSubsystem.angleSync(targetAngle);

    SmartDashboard.putString("shooter/autoZone", zoneLabel);
    SmartDashboard.putNumber("shooter/autoAngle", targetAngle);
    SmartDashboard.putNumber("shooter/autoVelocity", targetVelocity);
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
    angleSubsystem.lockCurrentAngle();
  }
}