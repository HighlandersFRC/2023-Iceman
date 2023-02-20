// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.IntakeDefaultCommand;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final TalonFX grabberMotor = new TalonFX(15, "Canivore");
  // private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(10);
  private final VoltageOut percentRequest = new VoltageOut(-6);
  

  public Intake() {}

  public void init() {
    // grabberMotor.configFactoryDefault();
    // grabberMotor.setNeutralMode(NeutralMode.Brake);
    // grabberMotor.configOpenloopRamp(0.25);
    // // grabberMotor.ramp
    // grabberMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 1, 1, 0));
    
    
    setDefaultCommand(new IntakeDefaultCommand(this));
  }

  public void setGrabberMotorMaxPercent(double percent) {
    System.out.println("Runnging: " + percent + " percent max");
    // grabberMotor.setControl(this.torqueRequest.withOutput(10));
    grabberMotor.setControl(this.percentRequest.withOutput(percent));
  }

  // public double getGrabberMotorCurrent() {
  //   return grabberMotor.getStatorCurrent();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
