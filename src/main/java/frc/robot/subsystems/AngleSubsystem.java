// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import java.util.function.DoubleSupplier;

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
  private DoubleSupplier distanceSupplierCm = () -> 0.0;

  /** Creates a new AngleSubsystem. */
  public AngleSubsystem() {
    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig.idleMode(IdleMode.kBrake);
    angleConfig.encoder.positionConversionFactor(45); //把圈數轉成自己想要的角度（角度??）
    angleConfig.softLimit.forwardSoftLimitEnabled(true);
    angleConfig.softLimit.forwardSoftLimit(AngleConstants.angleMotorMaxAngle);

    angleConfig.softLimit.reverseSoftLimitEnabled(true);
    angleConfig.softLimit.reverseSoftLimit(AngleConstants.angleMotorMinAngle);

    angleConfig.inverted(AngleConstants.angleInverted);
    angleConfig.closedLoop.pid(
        AngleConstants.angleMotorKp,
        AngleConstants.angleMotorKi,
        AngleConstants.angleMotorKd);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  public double getCurrentTargetAngle() {
    return this.targetAngle;
  }

  // Angle Motor Sync
  public void angleSync(double targetAngle) {
    double currentAngle = angleEncoder.getPosition();
    this.targetAngle = targetAngle;
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
    double targetAngle = preset.getAngle(this);
    this.targetAngle = targetAngle;
    Command cmd = run(() -> angleSync(targetAngle))
        .until(() -> Math.abs(angleEncoder.getPosition() - targetAngle) <= AngleConstants.angleTolerance);
    cmd.setName("angleLocatedTo" + preset.name() + "Cmd");
    return cmd;
  }
  // AdjustSpeedAngle 會使用這個方法，將目前離 hub 的距離傳入 AngleSubsystem，讓 AngleSubsystem
  // 可以計算出自動角度 (Auto Angle)。

  /** 由外部 (AdjustSpeedAngle) 每個 loop 呼叫，更新目前離 hub 的距離. */
  public void setDistanceSupplier(DoubleSupplier distanceSupplierCm) {
    this.distanceSupplierCm = distanceSupplierCm;
  }

  public double getAutoAngle() {
    return MathUtil.clamp(
        AngleConstants.angleDistanceMultiplier
            * Math.exp(AngleConstants.angleDistanceExponent * distanceSupplierCm.getAsDouble()),
        AngleConstants.angleMotorMinAngle,
        AngleConstants.angleMotorMaxAngle);
  }

  public enum AnglePreset {
    /** 最大角度 (Max Angle). */
    MAX(() -> AngleConstants.angleMotorMaxAngle),
    /** 傳輸角度 (Trans Angle). */
    TRANS(() -> AngleConstants.angleMotorTransAngle),
    /** 射球角度 (Shoot Angle). */
    SHOOT(() -> AngleConstants.angleMotorShootAngle),
    /** 最小角度 (Min Angle). */
    CLOSE(() -> AngleConstants.angleMotorMinAngle),
    /** 自動角度 (Auto Angle). */
    AUTO(null); // 特殊處理，見下方 getAngle()

    private final DoubleSupplier angleSupplier;

    AnglePreset(DoubleSupplier angleSupplier) {
      this.angleSupplier = angleSupplier;
    }

    public double getAngle(AngleSubsystem subsystem) {
      if (this == AUTO) {
        return subsystem.getAutoAngle();
      }
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