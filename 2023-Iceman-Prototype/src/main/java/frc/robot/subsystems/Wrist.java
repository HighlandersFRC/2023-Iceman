// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.WristDefaultCommand;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  private final CANSparkMax wristMotor = new CANSparkMax(13, MotorType.kBrushless);
  // private final CANSparkMax grabberMotor = new CANSparkMax(14, MotorType.kBrushless);

  private final TalonFX grabberMotor = new TalonFX(15);

  public Wrist() {}

  public void init() {
    setDefaultCommand(new WristDefaultCommand(this));
  }

  public double getGrabberMotorCurrent() {
    return grabberMotor.getOutputCurrent();
  }

  public void setWristMotorPercent(double percent) {
    wristMotor.set(percent);
  }

  public void setGrabberMotorPercent(double percent) {
    grabberMotor.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
