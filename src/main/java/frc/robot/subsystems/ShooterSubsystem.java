// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1 = new SparkMax(ShooterConstants.shooterMotorID1, MotorType.kBrushless);
  private final SparkMax shooterMotor2 = new SparkMax(ShooterConstants.shooterMotorID2, MotorType.kBrushless);
  private final SparkMax complexMotor1 = new SparkMax(ShooterConstants.complexMotorID1, MotorType.kBrushless);
  private final SparkMax complexMotor2 = new SparkMax(ShooterConstants.complexMotorID2, MotorType.kBrushless);
  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.shooterFeedforwardKs,
      ShooterConstants.shooterFeedforwardKv,
      ShooterConstants.shooterFeedforwardKa);

  private final SlewRateLimiter shooterRateLimiter = new SlewRateLimiter(800);

  private RelativeEncoder shooterEncoder;
  private RelativeEncoder complexEncoder;

  private double shooterTargetVelocity = 0;

  // shooter = 射球的馬達(Up)
  // complex = 介在傳輸和射球之間的馬達(Down)
  // transport = 傳輸的馬達

  public ShooterSubsystem() {
    // Up Motor follow Down Motor
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();
    shooterConfig.inverted(ShooterConstants.shooterUpMotorInverted);
    shooterFollowerConfig.follow(ShooterConstants.shooterMotorID1, false); // 馬達平行裝
    shooterConfig.smartCurrentLimit(ShooterConstants.shooterCurrentLimit);
    shooterMotor1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig complexConfig = new SparkMaxConfig();
    SparkMaxConfig complexFollowerConfig = new SparkMaxConfig();
    complexConfig.inverted(ShooterConstants.shooterDownMotorInverted);
    complexFollowerConfig.follow(ShooterConstants.complexMotorID1, false); // 馬達平行裝
    complexConfig.smartCurrentLimit(ShooterConstants.complexCurrentLimit);
    complexMotor1.configure(complexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    complexMotor2.configure(complexFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Shooter
  private void setShooterVoltage(double targetVelocity) {
    double feedforwardVoltage = shooterFeedforward.calculate(targetVelocity);
    // 計算前饋電壓，需要去測看看會不會每顆馬達都不一樣的前饋電壓
    shooterMotor1.setVoltage(feedforwardVoltage);
    complexMotor1.setVoltage(feedforwardVoltage);
  }

  public void shoot() {
    double target = shooterRateLimiter.calculate(ShooterConstants.shooterNominalTarget);
    setShooterVoltage(target);
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


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("shooter/shooterAtSpeed", isShooterAtSpeed());
    SmartDashboard.putNumber("shooter/shooterRPM", getShooterVelocity());
    SmartDashboard.putNumber("shooter/complexRPM", getComplexVelocity());
    SmartDashboard.putNumber("shooter/targetRPM", shooterTargetVelocity);
    SmartDashboard.putData("shooter/subsystem", this);
  }
}
