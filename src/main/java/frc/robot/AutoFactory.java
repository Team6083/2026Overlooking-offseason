// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class AutoFactory {
    private static AutoFactory autofactory;

    public static void configureAutoBuilder(SwerveDrive swerveDrive) {
        autofactory = new AutoFactory(
            swerveDrive::getPose2d,
            swerveDrive::resetPose,
            swerveDrive::followSample,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            swerveDrive
        );
    }
}
