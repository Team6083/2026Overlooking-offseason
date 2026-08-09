// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldZones;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdjustSpeedAngle extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final SwerveDrive swerveDrive;

  private double targetVelocity;
  private double targetAngle;

  /** Creates a new AdjustSpeedAngle. */
  public AdjustSpeedAngle(ShooterSubsystem shooterSubsystem,
      AngleSubsystem angleSubsystem,
      SwerveDrive swerveDrive) {
    this.shooterSubsystem = shooterSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.swerveDrive = swerveDrive;
    addRequirements(shooterSubsystem, angleSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d robotPos = swerveDrive.getPose2d().getTranslation();
    Distance dis = Meters.of(robotPos.getDistance(getHubPosition()));
    boolean inTrench = FieldZones.trenchZoneWithMargin.contains(robotPos);

    targetVelocity = MathUtil.clamp(ShooterConstants.shooterDistanceMultiplier
        * Math.exp(ShooterConstants.shooterDistanceExponent * dis.in(Centimeters)),
        0.0, ShooterConstants.maxShooterVelocity); // 2207.31e^0.0017x

    if (inTrench) {
      // 過trench/bump時角度壓到最低，避免卡到障礙
      targetAngle = AngleConstants.angleMotorMinAngle;
    } else {
      targetAngle = MathUtil.clamp(AngleConstants.angleDistanceMultiplier
          * Math.exp(AngleConstants.angleDistanceExponent * dis.in(Centimeters)),
          AngleConstants.angleMotorMinAngle, AngleConstants.angleMotorMaxAngle); // 待測公式
    }

    shooterSubsystem.shoot(targetVelocity + 100);
    angleSubsystem.angleSync(targetAngle);

    SmartDashboard.putNumber("shooterDistance", dis.in(Centimeters));
    SmartDashboard.putNumber("shooterTargetAngle", targetAngle);
    SmartDashboard.putBoolean("shooter/inTrench", inTrench);
  }

  private double getHubPositionX() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubX;
    }
    return FieldConstants.blueHubX;
  }

  private double getHubPositionY() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return FieldConstants.redHubY;
    }
    return FieldConstants.blueHubY;
  }

  private Translation2d getHubPosition() {
    return new Translation2d(getHubPositionX(), getHubPositionY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    angleSubsystem.lockCurrentAngle();
  }
}