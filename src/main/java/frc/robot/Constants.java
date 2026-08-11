// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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

  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;
    public static final int pivotMotorId = 35;

    public static final double pivotExpectedZero = 0;

    public static final double pivotEncoderFullRange = 360;
    public static final double pivotDeployStopPosition = 4;
    public static final double pivotRetractStopPosition = 0.1;
    public static final double pivotRetakeStopPosition = 30;

    public static final double pivotMaxOutput = 1.8;
    public static final double pivotRetakeMaxOutput = 0.05;

    public static final boolean intakeInverted = false;
    public static final boolean pivotInverted = true;

    public static final double intakeSpeed = 0.65;
    public static final double reverseIntakeSpeed = -0.65;

    public static final double pivotManualSpeed = 0.3;
    public static final double pivotManualRetractSpeed = 0.3;
    public static final double PivotRetakeSpeed = -0.2;

    public static final double pivotFollowKp = 0.03;
    public static final double pivotFollowKi = 0;
    public static final double pivotFollowKd = 0;
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
}