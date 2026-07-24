// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterUpMotor1 = new SparkMax(ShooterConstants.shooterUpMotorID1, MotorType.kBrushless);
  private final SparkMax shooterUpMotor2 = new SparkMax(ShooterConstants.shooterUpMotorID2, MotorType.kBrushless);
  private final SparkMax shooterDownMotor1 = new SparkMax(ShooterConstants.shooterDownMotorID1, MotorType.kBrushless);
  private final SparkMax shooterDownMotor2 = new SparkMax(ShooterConstants.shooterDownMotorID2, MotorType.kBrushless);
  private final SparkMax angleMotor = new SparkMax(ShooterConstants.angleMotorID, MotorType.kBrushless);

  private double targetVelocity = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(ShooterConstants.shooterUpMotorID1);
    shooterUpMotor2.configure(config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig config2 = new SparkMaxConfig();
    config2.follow(ShooterConstants.shooterDownMotorID1);
    shooterDownMotor2.configure(config2,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void shoot() {
  }

  public void stopshoot() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
