// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;

public class AngleSubsystem extends SubsystemBase {
  private final SparkMax angleMotor = new SparkMax(AngleConstants.angleMotorID, MotorType.kBrushless);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(
      AngleConstants.angleFeedforwardKs,
      AngleConstants.angleFeedforwardKg,
      AngleConstants.angleFeedforwardKa,
      AngleConstants.angleFeedforwardKv);
  private final PIDController angleFollowPIDController = new PIDController(
      AngleConstants.angleMotorKp,
      AngleConstants.angleMotorKi,
      AngleConstants.angleMotorKd);

  private RelativeEncoder angleEncoder = angleMotor.getEncoder();

  private final SparkClosedLoopController angleController = angleMotor.getClosedLoopController();
  private double targetAngle = 0;

  /** Creates a new AngleSubsystem. */
  public AngleSubsystem() {
    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig.idleMode(IdleMode.kBrake);
    angleConfig.smartCurrentLimit(AngleConstants.angleFreeLimit, AngleConstants.angleStallLimit);
    angleConfig.encoder.positionConversionFactor(360.0 / 10);
    angleConfig.softLimit.forwardSoftLimitEnabled(true);
    angleConfig.softLimit.forwardSoftLimit(AngleConstants.angleMotorMaxAngle);

    angleConfig.softLimit.reverseSoftLimitEnabled(false);
    angleConfig.softLimit.reverseSoftLimit(AngleConstants.angleMotorMinAngle);

    angleConfig.inverted(AngleConstants.angleInverted);
    angleConfig.closedLoop.pid(
        AngleConstants.angleMotorKp,
        AngleConstants.angleMotorKi,
        AngleConstants.angleMotorKd);

    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPosition(AngleConstants.angleExpectedZero);

    setDefaultCommand(holdAngleCmd());
  }

  private double getAngleDegree() {
    return angleEncoder.getPosition();
  }

  // Angle Motor
  public void angleMotor(double voltage) {
    angleMotor.setVoltage(voltage);
  }

  public void stopAngleMotor() {
    this.targetAngle = 0;
    angleMotor(0);
  }

  public void lockCurrentAngle() {
    this.targetAngle = angleEncoder.getPosition();
  }

  // Angle Motor Sync
  public void angleSync(double targetAngle) {
    double currentAngle = angleEncoder.getPosition();
    double pidOutput = angleFollowPIDController.calculate(currentAngle, targetAngle);
    // WPILib 的 ArmFeedforward.calculate 預設是接收 弧度 (Radians)
    double ffOutput = armFeedforward.calculate(
        Math.toRadians(currentAngle),
        0);
    angleMotor(pidOutput + ffOutput);
  }

  // Angle commands
  public Command angleMotorCmd(double voltage) {
    Command cmd = runEnd(() -> angleMotor(voltage), this::stopAngleMotor);
    cmd.setName("angleMotor+" + voltage + "Cmd");
    return cmd;
  }

  // Angle Sync command
  public Command angleSyncCmd(double targetAngle) {
    return run(() -> angleSync(targetAngle))
        .finallyDo(() -> lockCurrentAngle())
        .withName("angleSync+" + targetAngle + "Cmd");
  }

  public Command holdAngleCmd() {
    return run(() -> angleSync(this.targetAngle))
        .withName("holdAngleCmd");
  }

  public Command adjustAngleCmd(AnglePreset preset) {
    double targetAngle = preset.getAngle();
    this.targetAngle = targetAngle;
    Command cmd = run(() -> angleSync(targetAngle))
        .until(() -> Math.abs(angleEncoder.getPosition() - targetAngle) <= AngleConstants.angleTolerance)
        .finallyDo(() -> this.targetAngle = targetAngle); 
    cmd.setName("angleLocatedTo" + preset.name() + "Cmd");
    return cmd;
  }

  public static double getAutoAngle() {
    return AngleConstants.angleMotorShootAngle;
  }

  public enum AnglePreset {
    /** 傳輸角度 (Max Angle). */
    TRANS(() -> AngleConstants.angleMotorMaxAngle),

    /** 射球角度 (Shoot Angle). */
    SHOOT(() -> AngleConstants.angleMotorShootAngle),

    /** 歸位角度 (Min Angle),過trench使用. */
    CLOSE(() -> AngleConstants.angleMotorMinAngle),

    /** 自動追蹤角度 (動態計算). */
    AUTO(AngleSubsystem::getAutoAngle);

    private final DoubleSupplier angleSupplier;

    AnglePreset(DoubleSupplier angleSupplier) {
      this.angleSupplier = angleSupplier;
    }

    // 取得當前即時的角度值
    public double getAngle() {
      return angleSupplier.getAsDouble();
    }
  }

  // Trans 100%
  // Shoot 30%
  // Auto ??%
  // Close 00%

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/angleDegree", getAngleDegree());
    SmartDashboard.putNumber("shooter/angleTargetSet", targetAngle);
    SmartDashboard.putNumber("shooter/angleTarget", angleMotor.getOutputCurrent());
  }
}
