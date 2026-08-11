// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.zone.CompositeZone;
import frc.robot.lib.zone.RectZone;
import frc.robot.lib.zone.Zone;

/** Add your docs here. */
public class Constants {
  public record SwerveModuleConstant(
      int turningMotorId,
      int driveMotorId,
      int canCoderId,
      double canCoderOffset,
      boolean turningInverted,
      boolean driveInverted,
      String name) {
  }

  public record DriveBaseConstant(
      SwerveModuleConstant frontLeft,
      SwerveModuleConstant frontRight,
      SwerveModuleConstant backLeft,
      SwerveModuleConstant backRight) {
  }

  public static final class ModuleConstant {
    // define the radius of the wheel in meters
    public static final Distance kWheelRadius = Inches.of(2);
    public static final LinearVelocity kMaxModuleSpeed = MetersPerSecond.of(4);
  }

  public static final class SwerveControlConstants {
    public static final double kFastMagnification = 0.6;
    public static final double kSlowMagnification = 0.3;
    public static final double kFastRotMagnification = 0.8;
    public static final double kSlowRotMagnification = 0.4;
  }

  public static final DriveBaseConstant TEST_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          23, 22, 11, 0.338623, true, false, "FrontLeft"),
      new SwerveModuleConstant(
          20, 21, 13, 0.320068, true, false, "FrontRight"),
      new SwerveModuleConstant(
          26, 27, 12, 0.406250, true, false, "BackLeft"),
      new SwerveModuleConstant(
          24, 25, 14, -0.239990, true, false, "BackRight"));
  public static final DriveBaseConstant COMPETITION_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          20, 21, 12, 0.348145, true, false, "FrontLeft"),
      new SwerveModuleConstant(
          22, 23, 14, 0.482666, true, false, "FrontRight"),
      new SwerveModuleConstant(
          26, 27, 11, 0.403809, true, false, "BackLeft"),
      new SwerveModuleConstant(
          24, 25, 13, -0.242676, true, false, "BackRight"));

  public static final class VisionConstant {
    public static final double minQualityThreshold = 0.3;
    public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.9, 0.9, Math.toRadians(10));
    public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.3, 0.3, Math.toRadians(5));

    public static final int botposeX = 0;
    public static final int botposeY = 1;
    public static final int botposeZ = 2;
    public static final int botposeRoll = 3;
    public static final int botposePitch = 4;
    public static final int botposeYaw = 5;
    public static final int botposeLatency = 6;
    public static final int botposeTagCount = 7;
    public static final int botposeAvgDist = 9;
    public static final int botposeAvgArea = 10;
    public static final int botposeLength = 11;

    public static final int rawfiducialsStride = 7;
    public static final int rfId = 0;
    public static final int rfTxnc = 1;
    public static final int rfTync = 2;
    public static final int rfTa = 3;
    public static final int rfAmbiguity = 6;

    public static final double areaReference = 0.8;
    public static final double baseTranslationStd = 0.1;

    public static final double untrustedStd = 9999.0;
  }

  public static final class TransportConstants {
    public static final int transportMotorID = 36;
    public static final double transportMoterIn = 0.5;
    public static final double transportMoterOut = -0.5;
    public static final boolean transportMotorInverted = true;
  }

  public final class FeederConstants {
    public static final int feederMotorId = 32;
    public static final boolean feederMotorInverted = false;
    public static final double feederMotorIn = 0.5;
    public static final double feederMotorOut = -0.5;
  }

  public static class FieldConstants {
    public static final double blueHubX = 4.611624;
    public static final double blueHubY = 4.021328;
    public static final double redHubX = 11.901424;
    public static final double redHubY = 4.021328;

    public static final double blueTrenchMinX = 3.65;
    public static final double blueTrenchMaxX = 5.65;
    public static final double redTrenchMinX = 10.95;
    public static final double redTrenchMaxX = 12.95;

    public static final double blueLeftTrenchMinY = 6.25;
    public static final double blueLeftTrenchMaxY = 8.25;
    public static final double blueRightTrenchMinY = -0.25;
    public static final double blueRightTrenchMaxY = 1.75;

    public static final Distance fieldWidth = Inches.of(317.69);
    public static final Distance fieldLength = Inches.of(651.22);

    public static final Distance trenchToWall = Inches.of(182.11);
    public static final Distance trenchWidth = Inches.of(50.35);

    public static final Distance trenchZoneLength = Meters.of(4);

    public static final double trenchAngleMargin = 0.3;

    public static Translation2d getHubPosition() {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return new Translation2d(FieldConstants.redHubX, FieldConstants.redHubY);
      }
      return new Translation2d(FieldConstants.blueHubX, FieldConstants.blueHubY);
    }
  }

  public static class FieldZones {
    public static final double blueTrenchZoneMinX = FieldConstants.trenchToWall
        .minus(FieldConstants.trenchZoneLength.div(2)).in(Meters);
    public static final double blueTrenchZoneMaxX = FieldConstants.trenchToWall
        .plus(FieldConstants.trenchZoneLength.div(2)).in(Meters);

    public static final double redTrenchZoneMinX = FieldConstants.fieldLength
        .minus(FieldConstants.trenchToWall)
        .minus(FieldConstants.trenchZoneLength.div(2)).in(Meters);
    public static final double redTrenchZoneMaxX = FieldConstants.fieldLength
        .minus(FieldConstants.trenchToWall)
        .plus(FieldConstants.trenchZoneLength.div(2)).in(Meters);

    public static final double leftTrenchZoneMinY = FieldConstants.fieldWidth
        .minus(FieldConstants.trenchWidth).in(Meters);
    public static final double leftTrenchZoneMaxY = FieldConstants.fieldWidth.in(Meters) + 2;

    public static final double rightTrenchZoneMinY = -2;
    public static final double rightTrenchZoneMaxY = FieldConstants.trenchWidth.in(Meters);

    public static final Zone blueLeftZone = new RectZone(
        blueTrenchZoneMinX, blueTrenchZoneMaxX, leftTrenchZoneMinY, leftTrenchZoneMaxY);
    public static final Zone blueRightZOne = new RectZone(
        blueTrenchZoneMinX, blueTrenchZoneMaxX, rightTrenchZoneMinY, rightTrenchZoneMaxY);
    public static final Zone redLeftZone = new RectZone(
        redTrenchZoneMinX, redTrenchZoneMaxX, leftTrenchZoneMinY, leftTrenchZoneMaxY);
    public static final Zone redRightZone = new RectZone(
        redTrenchZoneMinX, redTrenchZoneMaxX, rightTrenchZoneMinY, rightTrenchZoneMaxY);

    public static final CompositeZone trenchZone = new CompositeZone(
        blueLeftZone, blueRightZOne, redLeftZone, redRightZone);

    // 放大trench空間讓angle可以有誤差區和下降時間
    private static final double margin = FieldConstants.trenchAngleMargin;

    public static final Zone blueLeftZoneWithMargin = new RectZone(
        blueTrenchZoneMinX - margin, blueTrenchZoneMaxX + margin,
        leftTrenchZoneMinY - margin, leftTrenchZoneMaxY + margin);
    public static final Zone blueRightZoneWithMargin = new RectZone(
        blueTrenchZoneMinX - margin, blueTrenchZoneMaxX + margin,
        rightTrenchZoneMinY - margin, rightTrenchZoneMaxY + margin);
    public static final Zone redLeftZoneWithMargin = new RectZone(
        redTrenchZoneMinX - margin, redTrenchZoneMaxX + margin,
        leftTrenchZoneMinY - margin, leftTrenchZoneMaxY + margin);
    public static final Zone redRightZoneWithMargin = new RectZone(
        redTrenchZoneMinX - margin, redTrenchZoneMaxX + margin,
        rightTrenchZoneMinY - margin, rightTrenchZoneMaxY + margin);

    public static final CompositeZone trenchZoneWithMargin = new CompositeZone(
        blueLeftZoneWithMargin, blueRightZoneWithMargin,
        redLeftZoneWithMargin, redRightZoneWithMargin);
  }

  public static final class ShooterConstants {
    public static final int shooterMotorID1 = 28;
    public static final int shooterMotorID2 = 29;
    public static final int complexMotorID1 = 30;
    public static final int complexMotorID2 = 31;

    public static final boolean shooterUpMotorInverted = true;
    public static final boolean shooterDownMotorInverted = false;

    public static final double shooterFeedforwardKs = 0.01; // 起始電壓
    public static final double shooterFeedforwardKv = 0.00207; // 速度電壓
    public static final double shooterFeedforwardKa = 0; // 加速度電壓

    public static final int shooterCurrentLimit = 40; // NEO 550 用 20，NEO/Vortex 常見 40~60
    public static final int complexCurrentLimit = 40;

    public static final double shooterNominalTarget = 3000; // 上方轉速
    public static final double complexNominalTarget = 3000; // 下方轉速

    public static final double shooterLowGearTarget = 1500;

    public static final double shooterDistanceMultiplier = 2207.31;
    public static final double shooterDistanceExponent = 0.0017;
    public static final double maxShooterVelocity = 5000;
    public static final double passVelocity = 2500;

    public static final double shooterVelocityTolerance = 100; // RPM，待實測調整

    public static final double shooterAccelLimit = 800;
  }

  public static final class AngleConstants {
    public static final int angleMotorID = 33;
    public static final boolean angleInverted = true;

    public static final int angleFreeLimit = 35; // 角度機構通常不需要太大電流
    public static final int angleStallLimit = 30;

    public static final double angleFeedforwardKs = 0.01; // 起始電壓
    public static final double angleFeedforwardKv = 0.00407; // 速度電壓
    public static final double angleFeedforwardKa = 0.02; // 加速度電壓
    public static final double angleFeedforwardKg = 0.04; // 重力電壓

    public static final double angleMotorMaxAngle = 55; // 最大角度
    public static final double angleMotorShootAngle = 25; // 初始角度(待測)
    public static final double angleMotorMinAngle = 0; // 最小角度
    public static final double angleMotorTransAngle = 45;

    public static final double angleMotorKp = 0.105;
    public static final double angleMotorKi = 0.0001;
    public static final double angleMotorKd = 0.002;

    public static final double angleTolerance = 0.45;

    public static final double angleExpectedZero = 1.1;

    public static final double angleDistanceMultiplier = 40;
    public static final double angleDistanceExponent = -0.001;
  }
}
