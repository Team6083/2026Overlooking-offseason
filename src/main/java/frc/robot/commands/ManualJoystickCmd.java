// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * 副手用左右搖桿手動控制:
 * - 左搖桿: 控制 angleSubsystem 角度 (比例控制, PID)
 * - 右搖桿: 控制 intakeSubsystem pivot 方向 (固定速度, 不改動IntakeSubsystem
 */
public class ManualJoystickCmd extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final CommandXboxController copilotController;

  private double targetAngle;

  private static final double deadLine = 0.1;
  private static final double maxDegreesPerSecond = 60;

  public ManualJoystickCmd(AngleSubsystem angleSubsystem, IntakeSubsystem intakeSubsystem,
      CommandXboxController copilotController) {
    this.intakeSubsystem = intakeSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.copilotController = copilotController;
    addRequirements(angleSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
    targetAngle = angleSubsystem.getCurrentTargetAngle();
  }

  @Override
  public void execute() {
    double rawLeftY = copilotController.getLeftY();
    double rawRightY = copilotController.getRightY();

    double stickAngle = MathUtil.applyDeadband(rawLeftY, deadLine);
    double stickPivot = MathUtil.applyDeadband(rawRightY, deadLine);

    // Angle
    double angleDelta = -stickAngle * maxDegreesPerSecond * 0.02;
    targetAngle += angleDelta;
    targetAngle = MathUtil.clamp(targetAngle,
        AngleConstants.angleMinManualAngle,
        AngleConstants.angleMotorMaxAngle);
    angleSubsystem.angleSync(targetAngle);

    // pivot
    if (stickPivot < 0) {
      // 搖桿上推 -> deploy 方向
      intakeSubsystem.manualPivotDeploy();
    } else if (stickPivot > 0) {
      // 搖桿下推 -> retract 方向
      intakeSubsystem.manualPivotReverse();
    } else {
      intakeSubsystem.stopRotate();
    }
  }

  @Override
  public void end(boolean interrupted) {
    angleSubsystem.lockCurrentAngle();
    intakeSubsystem.stopRotate();
  }
}