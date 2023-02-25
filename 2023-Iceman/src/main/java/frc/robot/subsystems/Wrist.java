// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.WristDefaultCommand;

public class Wrist extends SubsystemBase {

  private final CANSparkMax rotationMotor = new CANSparkMax(14, MotorType.kBrushless);

  private final SparkMaxAnalogSensor rotationAbsoluteEncoder = rotationMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

  private final double ABSOLUTE_ENCODER_ROTATION_MAX_VOLTAGE = 3.3;
  private final SparkMaxPIDController rotationPidController = rotationMotor.getPIDController();

  private double rotationSetPoint = 180;
  
  private double uprightOffset = 0;

  public Wrist() {}

  public void init() {
    rotationMotor.restoreFactoryDefaults();
    rotationMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    rotationMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    rotationMotor.setSmartCurrentLimit(40, 25);
    // rotationMotor.setSoftLimit(SoftLimitDirection.kForward, 63);
    // rotationMotor.setSoftLimit(SoftLimitDirection.kReverse, -63);
    rotationPidController.setFeedbackDevice(rotationAbsoluteEncoder);
    rotationPidController.setP(1.3);
    rotationPidController.setI(0);
    rotationPidController.setD(0.5);
    rotationPidController.setOutputRange(-1, 1);

    setDefaultCommand(new WristDefaultCommand(this));
  }

  public void setRotationMotorPercent(double percent) {
    rotationMotor.set(percent);
  }

  public double getWristMotorPosition() {
    return rotationMotor.getEncoder().getPosition();
  }

  public double getUprightOffset() {
    return uprightOffset;
  }

  public double getWristRotationPosition() {
    return 360 * (rotationAbsoluteEncoder.getPosition())/ABSOLUTE_ENCODER_ROTATION_MAX_VOLTAGE;
  }

  public double getAdustedWristRotation() {
    return getWristRotationPosition() - 180;
  }

  public void setWristRotationPosition(double position) {
    position = 180 + position;
    position = (position * ABSOLUTE_ENCODER_ROTATION_MAX_VOLTAGE)/360;
    rotationPidController.setReference(position, ControlType.kPosition);
  }

  public double getWristCurrent() {
    return rotationMotor.getAppliedOutput();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
