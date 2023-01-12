// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.ArmDefaultCommand;

public class MorrisArm extends SubsystemBase {
  /** Creates a new MorrisArm. */
  
  private CANSparkMax armMotor = new CANSparkMax(12, MotorType.kBrushless);

  public MorrisArm() {
  }

  public void init() {
    setDefaultCommand(new ArmDefaultCommand(this));
  }

  public void setArmMotorPercent(double percent) {
    armMotor.set(percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
