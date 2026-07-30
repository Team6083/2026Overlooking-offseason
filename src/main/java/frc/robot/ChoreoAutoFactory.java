// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class ChoreoAutoFactory {
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
        swerveDrive);
  }

  public AutoRoutine testAutoRoutine() {
    AutoRoutine routine = autofactory.newRoutine("testRoutine");

    AutoTrajectory Stright1m = routine.trajectory("Straight1m");
    AutoTrajectory Turn90 = routine.trajectory("Turn90");

    routine.active().onTrue(
        Commands.sequence(
            Stright1m.resetOdometry(),
            Stright1m.cmd()));
    Stright1m.done().onTrue(Turn90.cmd());

    return routine;
  }
}
