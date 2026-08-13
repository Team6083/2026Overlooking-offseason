// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class manualShooterComboCmd extends ParallelCommandGroup {
  public manualShooterComboCmd(
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      TransportSubsystem transportSubsystem,
      AngleSubsystem angleSubsystem) {
    addCommands(
        angleSubsystem.adjustAngleCmd(
            AngleSubsystem.AnglePreset.SHOOT),

        shooterSubsystem.shootCmd(),
         Commands.idle().until(shooterSubsystem::isShooterAtSpeed)
            .andThen(transportSubsystem.transportInCmd()
                .alongWith(feederSubsystem.feedInCmd())));
  }
}
