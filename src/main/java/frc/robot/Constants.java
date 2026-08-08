// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
          20, 21, 12, 0.348145, true, false, "FrontLeft"),
      new SwerveModuleConstant(
          22, 23, 14, 0.482666, true, false, "FrontRight"),
      new SwerveModuleConstant(
          26, 27, 11, 0.403809, true, false, "BackLeft"),
      new SwerveModuleConstant(
          24, 25, 13, -0.242676, true, false, "BackRight"));
  public static final DriveBaseConstant COMPETITION_CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          20, 21, 12, 0.348145, true, false, "FrontLeft"),
      new SwerveModuleConstant(
          22, 23, 14, 0.482666, true, false, "FrontRight"),
      new SwerveModuleConstant(
          26, 27, 11, 0.403809, true, false, "BackLeft"),
      new SwerveModuleConstant(
          24, 25, 13, -0.242676, true, false, "BackRight"));

  public static final class IntakeConstants {
    public static final int intakeMotorId = 34;
    public static final int pivotMotorId = 35;

    public static final double pivotExpectedZero = -271;

    public static final double pivotEncoderFullRange = 360;
    public static final double pivotDeployStopPosition = 93;
    public static final double pivotRetractStopPosition = 11;
    public static final double pivotRetakeStopPosition = 30;

    public static final double pivotMaxOutput = 1.0;
    public static final double pivotRetakeMaxOutput = 0.7;

    public static final boolean intakeInverted = true;
    public static final boolean pivotInverted = false;

    public static final double intakeSpeed = 0.65;
    public static final double reverseIntakeSpeed = -0.65;

    public static final double deployPivotSpeed = 0.8;
    public static final double retractPivotSpeed = -1;

    public static final double pivotManualSpeed = 0.4;

    public static final double pivotFollowKp = 0.03;
    public static final double pivotFollowKi = 0;
    public static final double pivotFollowKd = 0;

    public static final boolean motorLeftInverted = false;
    public static final boolean motorRightInverted = true;
    public static final boolean encoderLeftInverted = true;
    public static final boolean encoderRightInverted = false;
  }


}