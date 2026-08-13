// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.FieldUtil;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/**
 * 按下時: 不管在場上哪裡，角度一律切到SHOOT，轉速依距離動態計算。
 */
public class AdjustSpeedCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final SwerveDrive swerveDrive;

  private double targetVelocity;

  public AdjustSpeedCmd(ShooterSubsystem shooterSubsystem, AngleSubsystem angleSubsystem, SwerveDrive swerveDrive) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.swerveDrive = swerveDrive;
    addRequirements(shooterSubsystem, angleSubsystem);
  }

  @Override
  public void execute() {
    Distance dis = Meters.of(
        swerveDrive.getPose2d().getTranslation().getDistance(FieldUtil.getHubPosition()));

    targetVelocity = MathUtil.clamp(ShooterConstants.shooterDistanceMultiplier
        * Math.exp(ShooterConstants.shooterDistanceExponent * dis.in(Centimeters)),
        0.0, ShooterConstants.maxShooterVelocity);

    angleSubsystem.angleSync(AngleConstants.angleMotorShootAngle);
    shooterSubsystem.shoot(targetVelocity);

    SmartDashboard.putNumber("adjustSpeed/distance", dis.in(Centimeters));
    SmartDashboard.putNumber("adjustSpeed/targetVelocity", targetVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    angleSubsystem.lockCurrentAngle();
  }
}