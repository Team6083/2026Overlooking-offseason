// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1 = new SparkMax(ShooterConstants.shooterMotorID1, MotorType.kBrushless);
  private final SparkMax shooterMotor2 = new SparkMax(ShooterConstants.shooterMotorID2, MotorType.kBrushless);
  private final SparkMax complexMotor1 = new SparkMax(ShooterConstants.complexMotorID1, MotorType.kBrushless);
  private final SparkMax complexMotor2 = new SparkMax(ShooterConstants.complexMotorID2, MotorType.kBrushless);
  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.shooterFeedforwardKs,
      ShooterConstants.shooterFeedforwardKv,
      ShooterConstants.shooterFeedforwardKa);

  private final SparkMax angleMotor = new SparkMax(ShooterConstants.angleMotorID, MotorType.kBrushless);
  private final PIDController pivotFollowPIDController = new PIDController(
      ShooterConstants.angleMotorKp,
      ShooterConstants.angleMotorKi,
      ShooterConstants.angleMotorKd);

  private RelativeEncoder shooterEncoder;
  private RelativeEncoder complexEncoder;
  private RelativeEncoder angleEncoder = angleMotor.getEncoder();

  private double shooterTargetVelocity = 0;
  private double targetAngle = 0;
  private double autoAngle = 0;

  // shooter = 射球的馬達(Up)
  // complex = 介在傳輸和射球之間的馬達(Down)
  // transport = 傳輸的馬達

  public ShooterSubsystem() {
    // Up Motor follow Down Motor
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    shooterConfig.inverted(ShooterConstants.shooterUpMotorInverted);
    shooterFollowerConfig.follow(ShooterConstants.shooterMotorID1, false); // 馬達平行裝
    shooterMotor1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig complexConfig = new SparkMaxConfig();
    SparkMaxConfig complexFollowerConfig = new SparkMaxConfig();
    complexConfig.inverted(ShooterConstants.shooterDownMotorInverted);
    complexFollowerConfig.follow(ShooterConstants.complexMotorID1, false); // 馬達平行裝
    complexMotor1.configure(complexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    complexMotor2.configure(complexFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterEncoder = shooterMotor1.getEncoder();
    complexEncoder = complexMotor1.getEncoder();
    angleEncoder = angleMotor.getEncoder();
    angleEncoder.setPosition(ShooterConstants.angleExpectedZero);
  }

  // Shooter
  private void setShooterVoltage(double voltage) { // 請勿直接使用這個
    double feedforwardVoltage = shooterFeedforward.calculate(voltage); // 計算前饋電壓，需要去測看看會不會每顆馬達都不一樣的前饋電壓
    shooterMotor1.setVoltage(feedforwardVoltage);
    complexMotor1.setVoltage(feedforwardVoltage);
  }

  public void shoot() {
    this.shooterTargetVelocity = ShooterConstants.shooterNominalTarget;
    setShooterVoltage(shooterTargetVelocity);
  }

  public void stopShooter() {
    shooterTargetVelocity = 0;
    shooterMotor1.setVoltage(0);
    complexMotor1.setVoltage(0);
  }

  // Getter
  private double getShooterVelocity() {
    return shooterEncoder.getVelocity();
  }

  private double getComplexVelocity() {
    return complexEncoder.getVelocity();
  }

  private double getAngleVelocity() {
    return angleEncoder.getPosition();
  }

  public boolean isShooterAtSpeed() {
    return getShooterVelocity() >= shooterTargetVelocity;
  }

  // Shooter commands (不打值會使用預設值)
  public Command shootCmd() {
    Command cmd = runEnd(() -> shoot(), this::stopShooter);
    cmd.setName("shoot" + ShooterConstants.shooterNominalTarget + "Cmd");
    return cmd;
  }

  public Command shootCmd(double targetVelocity) {
    Command cmd = runEnd(() -> setShooterVoltage(targetVelocity), this::stopShooter);
    cmd.setName("shoot+" + targetVelocity + "Cmd");
    return cmd;
  }

  // Angle Motor
  public void angleMotor(double targetAngle) {
    this.targetAngle = targetAngle;
    angleMotor.setVoltage(targetAngle);
  }

  public void stopAngleMotor() {
    this.targetAngle = 0;
    angleMotor.setVoltage(0);
  }

  // Angle Motor Sync
  public void angleSync(double targetAngle) {
    double currentAngle = angleEncoder.getPosition();
    double output = pivotFollowPIDController.calculate(currentAngle, targetAngle);
    angleMotor.setVoltage(output);
  }

  // Angle commands
  public Command angleMotorCmd(double targetAngle) {
    Command cmd = runEnd(() -> angleMotor(targetAngle), this::stopAngleMotor);
    cmd.setName("angleMotor+" + targetAngle + "Cmd");
    return cmd;
  }

  // Angle Sync command
  public Command angleSyncCmd(double targetAngle) {
    Command cmd = runEnd(() -> angleSync(targetAngle), this::stopAngleMotor);
    cmd.setName("angleSync+" + targetAngle + "Cmd");
    return cmd;
  }

  public Command angleLocatedTo(double targetAngle) {
    double tolerance = 0.5; // 容許誤差
    Command cmd = run(() -> angleSync(targetAngle))
        .until(() -> Math.abs(angleEncoder.getPosition() - targetAngle) <= tolerance)
        .finallyDo(interrupted -> stopAngleMotor());
    cmd.setName("angleLocatedTo" + targetAngle + "Cmd");
    return cmd;
  }

  public Command adjustAngleCmd(AnglePreset preset) {
    double targetAngle = preset.getAngle();
    this.targetAngle = targetAngle;
    return angleLocatedTo(targetAngle);
  }

  public static double getAutoAngle() {
    return ShooterConstants.angleMotorShootAngle;
  }

  public enum AnglePreset {
    TRANS(() -> ShooterConstants.angleMotorMaxAngle),
    SHOOT(() -> ShooterConstants.angleMotorShootAngle),
    CLOSE(() -> ShooterConstants.angleMotorMinAngle),
    AUTO(ShooterSubsystem::getAutoAngle);

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
    SmartDashboard.putBoolean("shooter/shooterAtSpeed", isShooterAtSpeed());
    SmartDashboard.putNumber("shooter/shooterRPM", getShooterVelocity());
    SmartDashboard.putNumber("shooter/complexRPM", getComplexVelocity());
    SmartDashboard.putNumber("shooter/angleDegree", getAngleVelocity());
    SmartDashboard.putNumber("shooter/targetRPM", shooterTargetVelocity);
    SmartDashboard.putNumber("shooter/angleTarget", targetAngle);
    SmartDashboard.putData("shooter/subsystem", this);
  }
}
