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
 * 副手用左搖桿手動控制角度:
 * - 搖桿往上推到底(<= -deadLine附近全速) -> 逼近最大角度
 * - 搖桿往下推到底 -> 逼近最低可用角度(10度，不是完全歸零，避免誤觸直接關到底損害shooter)
 * - 搖桿推多少幅度，角度變化速度就跟著多快(比例控制，不是選固定preset)
 */
public class ManualJoystickCmd extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private final AngleSubsystem angleSubsystem;
  private final CommandXboxController copilotController;

  private double targetAngleX;

  private double targetAngleY;

  private static final double deadLine = 0.1; // 搖桿死區，避免手把飄移誤觸發
  private static final double maxDegreesPerSecond = 60; // 搖桿推到底時，每秒最多轉多少度，需依機構實測調整
  private static final double loopPeriodSeconds = 0.02; // 標準 20ms loop

  public ManualJoystickCmd(AngleSubsystem angleSubsystem, IntakeSubsystem intakeSubsystem,
      CommandXboxController copilotController) {
    this.intakeSubsystem = intakeSubsystem;
    this.angleSubsystem = angleSubsystem;
    this.copilotController = copilotController;
    addRequirements(angleSubsystem);
  }

  @Override
  public void initialize() {
    targetAngleX = angleSubsystem.getCurrentTargetAngle(); // 從目前角度接續，不要突然跳
    targetAngleY = intakeSubsystem.getPivotPosition(); 
  }

  @Override
  public void execute() {
    double rawY = copilotController.getRightY(); // Xbox: 上推通常是負值
    double rawX = copilotController.getLeftY(); // Xbox: 上推通常是負值

    double stickValueY = MathUtil.applyDeadband(rawY, deadLine);
    double stickValueX = MathUtil.applyDeadband(rawX, deadLine);

    // 上推(負值)要對應角度增加，所以取負號
    double angleDeltaX = -stickValueX * maxDegreesPerSecond * loopPeriodSeconds;
    double angleDeltaY = -stickValueY * maxDegreesPerSecond * loopPeriodSeconds;

    targetAngleX += angleDeltaX;
    targetAngleY += angleDeltaY;

    targetAngleX = MathUtil.clamp(targetAngleX,
        AngleConstants.angleMinManualAngle, // = 10
        AngleConstants.angleMotorMaxAngle);

    if (targetAngleY > 0) {
      intakeSubsystem.deployPivotCmd();

    } else if (targetAngleY < 0) {
      intakeSubsystem.retractPivotCmd();
    }
    angleSubsystem.angleSync(targetAngleY);
  }

  @Override
  public void end(boolean interrupted) {
    angleSubsystem.lockCurrentAngle();
  }
}