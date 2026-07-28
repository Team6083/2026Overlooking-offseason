// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
  public static final class IntakeConstants {
  public static final int intakeTurningMotorId = 34;
  public static final int pivotMotorId = 30;
  public static final int pivotEncoderId = 3;

  public static final double pivotExpectedZero = -271;

  public static final double pivotEncoderFullRange = 360;
  public static final double pivotDeployStopPosition = 93;
  public static final double pivotRetractStopPosition = 11;

  public static final boolean intakeInverted = true;

  public static final double intakeSpeed = 0.65;
  public static final double reverseIntakeSpeed = -0.65;

  public static final double deployPivotSpeed = 0.8;
  public static final double retractPivotSpeed = -1;

  public static final double pivotManualSpeed = 0.4;

  public static final double pivotFollowKp = 0.03;
  public static final double pivotFollowKi = 0;
  public static final double pivotFollowKd = 0;

  public static final boolean motorLeftInverted = false;
  public static final boolean motorRightInverted = true;
  public static final boolean encoderLeftInverted = true;
  public static final boolean encoderRightInverted = false;
  }


}