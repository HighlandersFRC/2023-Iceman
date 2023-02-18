// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.IntakeDefaultCommand;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final WPI_TalonFX grabberMotor = new WPI_TalonFX(15);

  

  public Intake() {}

  public void init() {
    grabberMotor.configFactoryDefault();
    grabberMotor.setNeutralMode(NeutralMode.Brake);
    grabberMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 1, 1, 0));
    
    setDefaultCommand(new IntakeDefaultCommand(this));
  }

  public void setGrabberMotorPercent(double percent) {
    // if (grabberMotor.getStatorCurrent() > 20.0){
    //   grabberMotor.set(ControlMode.PercentOutput, 0.05);
    // } else {
    grabberMotor.set(ControlMode.PercentOutput, percent);
    // }
  }

  public double getGrabberMotorCurrent() {
    return grabberMotor.getStatorCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
