// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AngleConstants;
import frc.robot.subsystems.AngleSubsystem;

/**
 * 副手用左搖桿手動控制角度:
 * - 搖桿往上推到底(<= -deadband附近全速) -> 逼近最大角度
 * - 搖桿往下推到底 -> 逼近最低可用角度(10度，不是完全歸零，避免誤觸直接關到底損害shooter)
 * - 搖桿推多少幅度，角度變化速度就跟著多快(比例控制，不是選固定preset)
 */
public class ManualAngleJoystickCmd extends Command {
  private final AngleSubsystem angleSubsystem;
  private final CommandXboxController copilotController;

  private double targetAngle;

  private static final double deadband = 0.1; // 搖桿死區，避免手把飄移誤觸發
  private static final double maxDegreesPerSecond = 60; // 搖桿推到底時，每秒最多轉多少度，需依機構實測調整
  private static final double loopPeriodSeconds = 0.02; // 標準 20ms loop

  public ManualAngleJoystickCmd(AngleSubsystem angleSubsystem, CommandXboxController copilotController) {
    this.angleSubsystem = angleSubsystem;
    this.copilotController = copilotController;
    addRequirements(angleSubsystem);
  }

  @Override
  public void initialize() {
    targetAngle = angleSubsystem.getCurrentTargetAngle(); // 從目前角度接續，不要突然跳
  }

  @Override
  public void execute() {
    double rawY = copilotController.getLeftY(); // Xbox: 上推通常是負值

    double stickValue = MathUtil.applyDeadband(rawY, deadband);

    // 上推(負值)要對應角度增加，所以取負號
    double angleDelta = -stickValue * maxDegreesPerSecond * loopPeriodSeconds;

    targetAngle += angleDelta;

    // 限制範圍: 上限用機構最大角度，下限鎖在10度(避免搖桿誤觸直接歸零撞底損壞shooter)
    targetAngle = MathUtil.clamp(targetAngle,
        AngleConstants.angleMinManualAngle, // = 10
        AngleConstants.angleMotorMaxAngle);

    angleSubsystem.angleSync(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    angleSubsystem.lockCurrentAngle();
  }
}