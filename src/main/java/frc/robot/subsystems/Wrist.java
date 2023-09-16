// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.WristDefaultCommand;
import frc.robot.tools.controlloops.PID;

public class Wrist extends SubsystemBase {

  private final CANSparkMax rotationMotor = new CANSparkMax(23, MotorType.kBrushless);

  private final CANCoder rotationEncoder = new CANCoder(6, "Canivore");

  private double rotationSetPoint = 180;

  private PID pid;

  private double kP = 0.017;
  //kI MUST be 0
  private double kI = 0.0;
  private double kD = 0.0;
  
  private double uprightOffset = 0;

  public Wrist() {}

  public void init() {
    rotationMotor.restoreFactoryDefaults();
    rotationMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    rotationMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false);
    rotationMotor.setSmartCurrentLimit(40, 25);

    pid = new PID(kP, kI, kD);
    pid.setMaxOutput(0.9);
    pid.setMinOutput(-0.9);
    pid.setSetPoint(180);
    pid.updatePID(getWristRotationPosition());
    rotationMotor.set(0);

    rotationMotor.setIdleMode(IdleMode.kBrake);

    setDefaultCommand(new WristDefaultCommand(this));
  }

  public void teleopInit(){
    pid.setSetPoint(180);
    pid.updatePID(getWristRotationPosition());
    rotationMotor.set(0);
  }

  public void autoInit(){
    pid.setSetPoint(180);
    pid.updatePID(getWristRotationPosition());
    rotationMotor.set(0);
  }

  public void setRotationMotorPercent(double percent) {
    // if (getWristRotationPosition() >= 320 && percent < 0){
    //   rotationMotor.set(0);
    // } else if (getWristRotationPosition() <= 47 && percent > 0){
    //   rotationMotor.set(0);
    // } else {
    //   rotationMotor.set(percent);
    // }
    rotationMotor.set(percent);
  }

  public double getUprightOffset() {
    return uprightOffset;
  }

  public double getWristRotationPosition() {
    return rotationEncoder.getAbsolutePosition();
  }

  public void setRotationPosition(double position) {
    this.rotationSetPoint = position;
  }

  public double getWristCurrent() {
    return rotationMotor.getAppliedOutput();
  }

  @Override
  public void periodic() {
    this.pid.setSetPoint(this.rotationSetPoint);
    this.pid.updatePID(getWristRotationPosition());
    double result = pid.getResult();
    setRotationMotorPercent(result);
    // System.out.println("Wrist: " + rotationMotor.getEncoder().getPosition());
  }
}
