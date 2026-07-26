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
  public static final DriveBaseConstant CONFIG = new DriveBaseConstant(
      new SwerveModuleConstant(
          25, 27, 11, 0.337891, true, true, "FrontLeft"),
      new SwerveModuleConstant(
          26, 28, 13, 0.495361, true, true, "FrontRight"),
      new SwerveModuleConstant(
          24, 23, 12, -0.446289, true, true, "BackLeft"),
      new SwerveModuleConstant(
          22, 20, 14, 0.302979, true, true, "BackRight"));

  public static final class TransportConstants {
  public static final int transportMotorID = 0;
  public static final double transportMoterIn = 0.5;
  public static final double transportMoterOut = -0.5;
  public static final boolean transportMotorInverted = true;
}
}