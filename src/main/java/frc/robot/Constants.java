// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
          20, 21, 12, 0.348145, true, false, "FrontLeft"),
      new SwerveModuleConstant(
          22, 23, 14, 0.482666, true, false, "FrontRight"),
      new SwerveModuleConstant(
          26, 27, 11, 0.403809, true, false, "BackLeft"),
      new SwerveModuleConstant(
          22, 20, 14, 0.302979, true, true, "BackRight"));

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
  }
}
