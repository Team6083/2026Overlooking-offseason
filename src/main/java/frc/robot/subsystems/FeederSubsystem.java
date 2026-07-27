// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  /** Creates a new FeederSubsystem. */
  SparkMax feederMotor = new SparkMax(FeederConstants.feederMotorId, MotorType.kBrushless);

  public FeederSubsystem() {
  SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
  feederMotorConfig.inverted(FeederConstants.feederMotorInverted);
  }

  public void feedIn() {
    feederMotor.set(FeederConstants.feederMotorIn);;
  }

  public void feedOut() {
    feederMotor.set(FeederConstants.feederMotorOut);
  }

  public void feedStop() {
    feederMotor.set(0);
  }

  public Command feedInCmd() {
    Command cmd = runEnd(this::feedIn, this::feedStop);
    cmd.setName("feedInCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feeder/feederVoltage", feederMotor.getOutputCurrent());
    SmartDashboard.putData("feeder/subsystem", this);
     // This method will be called once per scheduler run
  } 
}
