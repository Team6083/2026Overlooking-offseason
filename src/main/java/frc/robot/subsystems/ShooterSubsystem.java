// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final SparkMax shooterUpMotor1 = new SparkMax(ShooterConstants.shooterUpMotorID1, MotorType.kBrushless);
  private final SparkMax shooterUpMotor2 = new SparkMax(ShooterConstants.shooterUpMotorID2, MotorType.kBrushless);
  private final SparkMax shooterDownMotor1 = new SparkMax(ShooterConstants.shooterDownMotorID1, MotorType.kBrushless);
  private final SparkMax shooterDownMotor2 = new SparkMax(ShooterConstants.shooterDownMotorID2, MotorType.kBrushless);
  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.shooterFeedforwardKs,
      ShooterConstants.shooterFeedforwardKv,
      ShooterConstants.shooterFeedforwardKa);

  private final SparkMax angleMotor = new SparkMax(ShooterConstants.angleMotorID, MotorType.kBrushless);

  private RelativeEncoder shooterUpEncoder;
  private RelativeEncoder shooterDownEncoder;
  private RelativeEncoder angleEncoder = angleMotor.getEncoder();

  private final PIDController pivotFollowPIDController = new PIDController(ShooterConstants.angleMotorKp,
      ShooterConstants.angleMotorKi, ShooterConstants.angleMotorKd);

  private double shooterTargetVelocity = 0;
  private double angleTargetVelocity = 0;

  // shooter = 射球的馬達(Up)
  // complex = 介在傳輸和射球之間的馬達(Down)
  // transport = 傳輸的馬達

  public ShooterSubsystem() {
    // Up Motor follow Down Motor
    SparkMaxConfig upConfig = new SparkMaxConfig();
    SparkMaxConfig upFollowerConfig = new SparkMaxConfig();
    upConfig.inverted(ShooterConstants.shooterUpMotorInverted);
    upFollowerConfig.follow(ShooterConstants.shooterUpMotorID1, false); // 馬達平行裝
    shooterUpMotor1.configure(upConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterUpMotor2.configure(upFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig downConfig = new SparkMaxConfig();
    SparkMaxConfig downFollowerConfig = new SparkMaxConfig();
    downConfig.inverted(ShooterConstants.shooterDownMotorInverted);
    downFollowerConfig.follow(ShooterConstants.shooterDownMotorID1, false); // 馬達平行裝
    shooterDownMotor1.configure(downConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterDownMotor2.configure(downFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterUpEncoder = shooterUpMotor1.getEncoder();
    shooterDownEncoder = shooterDownMotor1.getEncoder();
    angleEncoder = angleMotor.getEncoder();
  }

  // Shooter
  private void setShooterVoltage(double voltage) {
    double feedforwardVoltage = shooterFeedforward.calculate(voltage); // 計算前饋電壓，需要去測看看會不會每顆馬達都不一樣的前饋電壓
    shooterUpMotor1.setVoltage(feedforwardVoltage);
    shooterDownMotor1.setVoltage(feedforwardVoltage);
  }

  public void shoot(double targetVelocity) {
    this.shooterTargetVelocity = targetVelocity;
    setShooterVoltage(targetVelocity);
  }

  public void stopShooter() {
    setShooterVoltage(0);
    shooterUpMotor1.setVoltage(0);
    shooterDownMotor1.setVoltage(0);
  }

  // Getter
  private double getShooterVelocity() {
    return shooterUpEncoder.getVelocity();
  }

  public boolean isShooterAtSpeed() {
    return getShooterVelocity() >= shooterTargetVelocity;
  }

  // Shooter commands (不打值會使用預設值)
  public Command shootCmd() {
    Command cmd = runEnd(() -> shoot(ShooterConstants.shooterUpNominalTarget), this::stopShooter);
    cmd.setName("shoot" + ShooterConstants.shooterUpNominalTarget + "Cmd");
    return cmd;
  }

  public Command shootCmd(double targetVelocity) {
    Command cmd = runEnd(() -> shoot(targetVelocity), this::stopShooter);
    cmd.setName("shoot+" + targetVelocity + "Cmd");
    return cmd;
  }

  // Angle Motor
  public void angleMotor(double angleTargetVelocity) {
    this.angleTargetVelocity = angleTargetVelocity;
  }

  public void stopAngleMotor() {
    this.angleTargetVelocity = 0;
    angleMotor.setVoltage(0);
  }

  // Angle Motor Sync
  public void angleSync(double targetAngle) {
    double currentAngle = angleEncoder.getPosition();
    double output = pivotFollowPIDController.calculate(currentAngle, targetAngle);
    angleMotor.setVoltage(output);
  }

  // Angle commands
  public Command angleMotorCmd(double angleTargetVelocity) {
    Command cmd = runEnd(() -> angleMotor(angleTargetVelocity), this::stopAngleMotor);
    cmd.setName("angleMotor+" + angleTargetVelocity + "Cmd");
    return cmd;
  }

  // Angle Sync command
  public Command angleSyncCmd(double targetAngle) {
    Command cmd = runEnd(() -> angleSync(targetAngle), this::stopAngleMotor);
    cmd.setName("angleSync+" + targetAngle + "Cmd");
    return cmd;
  }

  // 傳入任意目標角度，轉到指定位置後自動結束 Command
  public Command angleLocatedTo(double targetAngle) {
    double tolerance = 0.5; // 容許誤差
    Command cmd = run(() -> angleSync(targetAngle))
        .until(() -> Math.abs(angleEncoder.getPosition() - targetAngle) <= tolerance)
        .finallyDo(interrupted -> stopAngleMotor());
    cmd.setName("goToAngle_" + targetAngle + "Cmd");
    return cmd;
  }

  public Command adjustAngleCmd(AnglePreset preset) {
    return angleLocatedTo(preset.angle);
  }

  public enum AnglePreset {
    TRANS(ShooterConstants.angleMotorMaxAngle),
    SHOOT(ShooterConstants.angleMotorShootAngle),
    // AUTO(), 還沒寫之後再填吧(?
    CLOSE(ShooterConstants.angleMotorMinAngle);

    public final double angle;

    AnglePreset(double angle) {
      this.angle = angle;
    }
  }

  // Trans
  // Shoot
  // Auto
  // Close
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter/shooterAtSpeed", isShooterAtSpeed());
    SmartDashboard.putNumber("shooter/shooterRPM", getShooterVelocity());
    SmartDashboard.putNumber("shooter/targetRPM", shooterTargetVelocity);
    SmartDashboard.putData("shooter/subsystem", this);
  }
}
