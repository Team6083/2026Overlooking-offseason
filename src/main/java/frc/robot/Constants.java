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

        public static final double shooterNominalTarget = 2000; // 上方轉速
        public static final double complexNominalTarget = 2000; // 下方轉速

        public static final double shooterLowGearTarget = 1500;
    }

    public static final class AngleConstants {
        public static final int angleMotorID = 33;
        public static final boolean angleInverted = true;

        public static final int angleFreeLimit = 30; // 角度機構通常不需要太大電流
        public static final int angleStallLimit = 20;

        public static final double angleFeedforwardKs = 0.01; // 起始電壓
        public static final double angleFeedforwardKv = 0.00407; // 速度電壓
        public static final double angleFeedforwardKa = 0.05; // 加速度電壓
        public static final double angleFeedforwardKg = 0.8; // 重力電壓

        public static final double angleMotorMaxAngle = 50; // 最大角度
        public static final double angleMotorShootAngle = 30; // 初始角度(待測)
        public static final double angleMotorMinAngle = 0; // 最小角度

        public static final double angleMotorKp = 0.12;
        public static final double angleMotorKi = 0;
        public static final double angleMotorKd = 0;

        public static final double angleTolerance = 0.2;

        public static final double angleExpectedZero = 0;
    }
}