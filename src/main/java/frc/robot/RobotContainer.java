// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FeederSubsystem;

public class RobotContainer {
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final FeederSubsystem feederSubsystem;
  public RobotContainer() {
    feederSubsystem = new FeederSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    mainController.a().onTrue(feederSubsystem.feedInCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
