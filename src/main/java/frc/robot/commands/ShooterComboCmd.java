// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class ShooterComboCmd extends Command {
  /** Creates a new ShooterComboCmd. */
  public ShooterComboCmd(
      ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsystem intakeSubsystem) {

    Command shoot = Commands.either(
        new CalculateSpeedShooterCmd(shooterSubsystem, swerveDrive),
        shooterSubsystem.shootCmd(),
        shouldSpeedDynamic);
    addCommands(
        shoot,
        Commands.idle().until(shooterSubsystem::isShooterAtSpeed)
            .andThen(transportSubsystem.transportInCmd()
                .alongWith(feederSubsystem.feedInCmd())
                .alongWith(intakeSubsystem.intakeCmd())));
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
