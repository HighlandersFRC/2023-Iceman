// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.ArmDefaultCommand;
import frc.robot.commands.defaults.IntakeDefault;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(12);
  private WPI_TalonFX intakeMotor2 = new WPI_TalonFX(11);
  private WPI_TalonFX intakeMotor3 = new WPI_TalonFX(20);

  public Intake() {
  }

  public void init() {
    intakeMotor.configFactoryDefault();
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor2.configFactoryDefault();
    intakeMotor2.setNeutralMode(NeutralMode.Brake);
    setDefaultCommand(new IntakeDefault(this));
  }

  public void setIntakePercent(double percent){
    intakeMotor.set(ControlMode.PercentOutput, percent);
    intakeMotor2.set(ControlMode.PercentOutput, -percent);
    intakeMotor3.set(ControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
