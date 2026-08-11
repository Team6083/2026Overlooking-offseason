// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * 依全場位置自動決定角度(副Driver，按著時持續跟隨):
 * | Trench (含緩衝) -> 角度歸零，方便過Trench/Bump
 * | Alliance zone -> 射擊角度(SHOOT)，由AutoFireCmd依距離覆寫實際目標角度
 * | Neutral zone -> 固定平飛角度(TRANS)
 */
public class AutoAngleCmd extends Command {
  private final AngleSubsystem angleSubsystem;
  private final SwerveDrive swerveDrive;

  public AutoAngleCmd(AngleSubsystem angleSubsystem, SwerveDrive swerveDrive) {
    this.angleSubsystem = angleSubsystem;
    this.swerveDrive = swerveDrive;
    addRequirements(angleSubsystem);
  }

  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);
    boolean inOwnZone = isInOwnZone(robotPos);

    double targetAngle;
    String zoneLabel;

    if (inTrench) {
      targetAngle = AngleConstants.angleMotorMinAngle;
      zoneLabel = "Trench";
    } else if (inOwnZone) {
      targetAngle = AngleConstants.angleMotorShootAngle;
      zoneLabel = "Alliance zone";
    } else {
      targetAngle = AngleConstants.angleMotorTransAngle;
      zoneLabel = "Neutral zone";
    }

    angleSubsystem.angleSync(targetAngle);

    SmartDashboard.putString("autoAngle/zone", zoneLabel);
    SmartDashboard.putNumber("autoAngle/target", targetAngle);
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
    angleSubsystem.lockCurrentAngle(); // 放開按鍵時鎖定在當下角度
  }
}