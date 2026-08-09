// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import frc.robot.Constants;
import frc.robot.Constants.DriveBaseConstant;

public class SwerveDriveFactory {
  public enum SwerveImplementation {
    YAGSL,
    WPILIB
  }

  public enum RobotVariant {
    TEST,
    COMPETITION
  }

  public static SwerveDrive createSwerveDrive(SwerveImplementation type, RobotVariant variant) {
    DriveBaseConstant driveBaseConstant = switch (variant) {
      case TEST -> Constants.TEST_CONFIG;
      case COMPETITION -> Constants.COMPETITION_CONFIG;
    };

    return switch (type) {
      case WPILIB -> new WpilibSwerveDrive(driveBaseConstant);
      default -> throw new UnsupportedOperationException("this type does not exist: " + type);
    };
  }
}