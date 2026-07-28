// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  //intake 是把球吸進去的部位，pivot 是把整個 intake 抬降的部位
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeTurningMotorId, MotorType.kBrushless);
  private final VictorSPX pivotMotor = new VictorSPX(IntakeConstants.pivotMotorId);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.pivotEncoderId,
      IntakeConstants.pivotEncoderFullRange, IntakeConstants.pivotExpectedZero);
  private final PIDController pivotFollowPIDController = new PIDController(IntakeConstants.pivotFollowKp,
      IntakeConstants.pivotFollowKi, IntakeConstants.pivotFollowKd);

  public IntakeSubsystem() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig.inverted(IntakeConstants.intakeInverted);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotMotor.setInverted(IntakeConstants.motorLeftInverted);
    pivotEncoder.setInverted(IntakeConstants.encoderLeftInverted);
    pivotFollowPIDController.enableContinuousInput(0, IntakeConstants.pivotEncoderFullRange);
  }

  // Intake
  public void intake() {
    intakeMotor.set(IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    intakeMotor.set(IntakeConstants.reverseIntakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }

  // Pivot
  //沒 PID 的，自己手動調到適合的角度
  public void manualPivotDeploy() {
    pivotMotor.set(ControlMode.PercentOutput, IntakeConstants.pivotManualSpeed);
  }

  public void manualPivotRetract() {
    pivotMotor.set(ControlMode.PercentOutput, -IntakeConstants.pivotManualSpeed);
  }

  public void stopRotate() {
    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  // Sync Pivot
  //有 PID 的，會轉到固定角度
  public void pivotDeploy() {
    runPivotTarget(IntakeConstants.pivotDeployStopPosition, IntakeConstants.pivotMaxOutput);
  }

  public void pivotRetract() {
    runPivotTarget(IntakeConstants.pivotRetractStopPosition, IntakeConstants.pivotMaxOutput);
  }
  public void pivotRetake() {
    runPivotTarget(IntakeConstants.pivotRetakeStopPosition, IntakeConstants.pivotRetakeMaxOutput);
  }

  private void runPivotTarget(double targetPosition, double maxOutput) {
    double currentPosition = pivotEncoder.get();
    double pidOutput = pivotFollowPIDController.calculate(currentPosition, targetPosition);
    pidOutput = MathUtil.clamp(pidOutput, -1.0, maxOutput);
    pivotMotor.set(ControlMode.PercentOutput, pidOutput);
  }

  // Getters
  public double getPivotPosition() {
    return pivotEncoder.get();
  }

  // intake
  public Command intakeCmd() {
    Command cmd = runEnd(this::intake, this::stopIntake);
    cmd.setName("intakeCmd");
    return cmd;
  }

  public Command reverseIntakeCmd() {
    Command cmd = runEnd(this::reverseIntake, this::stopIntake);
    cmd.setName("reverseIntakeCmd");
    return cmd;
  }

  // manual pivot
  public Command manualDeployPivotCmd() {
    Command cmd = runEnd(this::manualPivotDeploy, this::stopRotate);
    cmd.setName("manualDeployPivotCmd");
    return cmd;
  }
  
  public Command manualRetractPivotCmd() {
    Command cmd = runEnd(this::manualPivotRetract, this::stopRotate);
    cmd.setName("manualRetractPivotCmd");
    return cmd;
  }

  // auto pivot
  public Command deployPivotCmd() {
    Command cmd = runEnd(this::pivotDeploy, this::stopRotate);
    cmd.setName("deployPivotCmd");
    return cmd;
  }

  public Command retractPivotCmd() {
    Command cmd = runEnd(this::pivotRetract, this::stopRotate);
    cmd.setName("retractPivotCmd");
    return cmd;
  }

  public Command retakePivotCmd() {
    Command cmd = runEnd(this::pivotRetake, this::stopRotate);
    cmd.setName("retakePivotCmd");
    return cmd;
  }

  public Command autoDeployPivotCmd() {
    Command cmd = deployPivotCmd()
        .until(() -> getPivotPosition() >= IntakeConstants.pivotDeployStopPosition);
    cmd.setName("autoDeployPivotCmd");
    return cmd;
  }

  public Command autoRetractPivotCmd() {
    Command cmd = retractPivotCmd()
        .until(() -> getPivotPosition() <= IntakeConstants.pivotRetractStopPosition);
    cmd.setName("autoRetractPivotCmd");
    return cmd;
  }

  public Command autoRetakePivotCmd() {
    Command cmd = retakePivotCmd()
        .until(() -> getPivotPosition() >= IntakeConstants.pivotRetakeStopPosition);
    cmd.setName("autoRetakePivotCmd");
    return cmd;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake/motorVoltage", intakeMotor.getBusVoltage());
    SmartDashboard.putNumber("intake/pivotPositionDeg", getPivotPosition());
    SmartDashboard.putNumber("intake/pivotLeftVoltage", pivotMotor.getMotorOutputVoltage());
    SmartDashboard.putBoolean("intake/pivotLeftEncoderConnected", pivotEncoder.isConnected());
    SmartDashboard.putData("intake/subsystem", this);
    SmartDashboard.putData(pivotFollowPIDController);
  }
}