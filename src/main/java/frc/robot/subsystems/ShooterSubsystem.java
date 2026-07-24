// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterUpMotor1 = new SparkMax(ShooterConstants.shooterUpMotorID1, MotorType.kBrushless);
  private final SparkMax shooterUpMotor2 = new SparkMax(ShooterConstants.shooterUpMotorID2, MotorType.kBrushless);
  private final SparkMax shooterDownMotor1 = new SparkMax(ShooterConstants.shooterDownMotorID1, MotorType.kBrushless);
  private final SparkMax shooterDownMotor2 = new SparkMax(ShooterConstants.shooterDownMotorID2, MotorType.kBrushless);
  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.shooterFeedforwardKs,
      ShooterConstants.shooterFeedforwardKv, ShooterConstants.shooterFeedforwardKa);

  private final SparkMax angleMotor = new SparkMax(ShooterConstants.angleMotorID, MotorType.kBrushless);
  private final SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(ShooterConstants.angleFeedforwardKs,
      ShooterConstants.angleFeedforwardKv, ShooterConstants.angleFeedforwardKa);
  private RelativeEncoder shooterUpEncoder = shooterUpMotor1.getEncoder();
  private RelativeEncoder shooterDownEncoder = shooterDownMotor1.getEncoder();
  private double shooterTargetVelocity = 0;
  private double angleTargetVelocity = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    SparkMaxConfig upConfig = new SparkMaxConfig();
    upConfig.follow(ShooterConstants.shooterUpMotorID1);
    shooterUpMotor2.configure(upConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig downConfig = new SparkMaxConfig();
    downConfig.follow(ShooterConstants.shooterDownMotorID1);
    shooterDownMotor2.configure(downConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterUpMotor1.setInverted(ShooterConstants.shooterUpMotorInverted);
    shooterDownMotor1.setInverted(ShooterConstants.shooterDownMotorInverted);

    shooterUpEncoder = shooterUpMotor1.getEncoder();
    shooterDownEncoder = shooterDownMotor1.getEncoder();

    shooterUpMotor1.setVoltage(shooterTargetVelocity);
    angleMotor.setVoltage(angleTargetVelocity);
  }

  public void shoot(double shooterTargetVelocity) {
    this.shooterTargetVelocity = shooterTargetVelocity;
  }

  public void stopshoot() {
    this.shooterTargetVelocity = 0;
  }

  public void angleMotor(double angleTargetVelocity) {
    this.angleTargetVelocity = angleTargetVelocity;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
