// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveFactory;

public class RobotContainer {
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController = new CommandXboxController(0);
  private Supplier<Boolean> shouldSprint = () -> mainController.leftBumper().getAsBoolean();
  private Supplier<Boolean> shouldLockPose = () -> mainController.a().getAsBoolean();
   private final SendableChooser<Command> autoChooser;
   private  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public RobotContainer() {
    swerveDrive = SwerveDriveFactory.createSwerveDrive(
        SwerveDriveFactory.SwerveImplementation.WPILIB,
        SwerveDriveFactory.RobotVariant.TEST);


    Auto.configureAutoBuilder(swerveDrive);

    registerCommand();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("autoChooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController, shouldSprint, shouldLockPose));
    mainController.start().onTrue(Commands.runOnce(() -> {
      swerveDrive.zeroGyro();
      swerveDrive.resetPose(new Pose2d(swerveDrive.getPose2d().getTranslation(), Rotation2d.fromDegrees(0)));
    }));
  }

   private void registerCommand() {
    NamedCommands.registerCommand("Intake", Commands.runOnce(() -> intakeSubsystem.intake()));
    NamedCommands.registerCommand("ReverseIntake", Commands.runOnce(() -> intakeSubsystem.reverseIntake()));
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intakeSubsystem.stopIntake()));
    NamedCommands.registerCommand("DeployIntake", Commands.runOnce(() -> intakeSubsystem.deploy()));
    NamedCommands.registerCommand("RetractIntake", Commands.runOnce(() -> intakeSubsystem.retract()));
  
   }

   public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}