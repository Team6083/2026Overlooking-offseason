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
  public static final class ShooterConstants {
    public static final int shooterUpMotorID1 = 30;
    public static final int shooterUpMotorID2 = 31;
    public static final int shooterDownMotorID1 = 32;
    public static final int shooterDownMotorID2 = 33;

    public static final int angleMotorID = 34;

    public static final boolean shooterUpMotorInverted = false;
    public static final boolean shooterDownMotorInverted = true;

    public static final double shooterFeedforwardKs = 0.01;  //起始電壓
    public static final double shooterFeedforwardKv = 0.00207;  //速度電壓
    public static final double shooterFeedforwardKa = 0; //加速度電壓

    public static final double angleFeedforwardKs = 0.01;  //起始電壓
    public static final double angleFeedforwardKv = 0.00407;  //速度電壓
    public static final double angleFeedforwardKa = 0; //加速度電壓
    
    public static final double shooterUpNominalTarget = 3000; // 上方轉速
    public static final double shooterDownNominalTarget = 3000; // 下方轉速

    public static final double angleMotorKp = 0.1;
    public static final double angleMotorKi = 0.01;
    public static final double angleMotorKd = 0.001;
  }
}