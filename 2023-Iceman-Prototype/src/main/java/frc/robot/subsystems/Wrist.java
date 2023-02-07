// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.WristDefaultCommand;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  // private final CANSparkMax grabberMotor = new CANSparkMax(62, MotorType.kBrushless);
  // private final CANSparkMax rightPinchMotor = new CANSparkMax(25, MotorType.kBrushless);
  // private final CANSparkMax leftPinchMotor = new CANSparkMax(26, MotorType.kBrushless);
  private final WPI_TalonFX grabberMotor = new WPI_TalonFX(15);
  // private final CANSparkMax grabberMotor = new CANSparkMax(14, MotorType.kBrushless);

  // private final TalonFX wristMotor = new TalonFX(15);

  public Wrist() {}

  public void init() {
    grabberMotor.configFactoryDefault();
    grabberMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20.0, 20.0, 0.001));
    setDefaultCommand(new WristDefaultCommand(this));
  }

  public double getGrabberMotorCurrent() {
    return grabberMotor.getStatorCurrent();
  }

  public void setWristMotorPercent(double percent) {
    // wristMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setGrabberMotorPercent(double percent) {
    if (grabberMotor.getStatorCurrent() > 20.0){
      grabberMotor.set(ControlMode.PercentOutput, 0.05);
    } else {
      grabberMotor.set(ControlMode.PercentOutput, percent);
    }
    // rightPinchMotor.set(percent);
    // leftPinchMotor.set(-percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
