// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveDrive;

/** Add your docs here. */
public class ChoreoAutoFactory {
  private static AutoFactory autofactory;
  private static AutoChooser autoChooser;
  private static SwerveDrive swerveDrive;

  public static void configureAutoBuilder(SwerveDrive swerveDrive) {
    autofactory = new AutoFactory(
        swerveDrive::getPose2d,
        swerveDrive::resetPose,
        swerveDrive::followTrajectory,
        true,
        swerveDrive);
  }

  public static void configureAutoChooser() {
    autoChooser = new AutoChooser();

    autoChooser.addRoutine("testRoutine", ChoreoAutoFactory::testAutoRoutine);

    SmartDashboard.putData("Auto/ChoreoChooser", autoChooser);
  }

  public static AutoChooser getAutoChooser() {
    return autoChooser;
  }

  public static AutoRoutine testAutoRoutine() {
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
