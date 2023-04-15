// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.GroundCubeIntakeDefault;
import frc.robot.commands.defaults.IntakeDefaultCommand;

public class GroundCubeIntake extends SubsystemBase {
  /** Creates a new GroundCubeIntake. */

  private final TalonFX grabberMotor = new TalonFX(21, "Canivore");
  private final TalonFX rotationMotor = new TalonFX(20, "Canivore");
  
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(10, 0.1, 0, false);
  private final VoltageOut percentRequest = new VoltageOut(-6);

  private final PositionTorqueCurrentFOC positionRequest = new PositionTorqueCurrentFOC(0, 0.1, 0, true);
  
  private TalonFXConfigurator configurator = grabberMotor.getConfigurator();

  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  public GroundCubeIntake() {
  }

  public void init() {
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Add 0.05 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0; // A position error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0;
    
    configurator.apply(motorOutputConfigs);
    configurator.apply(slot0Configs);
    
    setDefaultCommand(new GroundCubeIntakeDefault(this));
  }

  public double getVelocity() {
    double velocity = grabberMotor.getVelocity().getValue();
    return velocity;
  }

  public double getPosition() {
    return Constants.getSideIntakeDegreesFromRotations(rotationMotor.getPosition().getValue());
  }

  public void setGrabberMotorMaxPercent(double percent) {
    // System.out.println("Runnging: " + percent + " percent max");
    // grabberMotor.setControl(this.torqueRequest.withOutput(10));
    grabberMotor.setControl(this.percentRequest.withOutput(percent));
  }

  public void setIntakeTorqueOutput(double amps, double maxPercent){
    grabberMotor.setControl(this.torqueRequest.withOutput(amps).withMaxAbsDutyCycle(maxPercent));
  }

  public void setIntakeRotation(double position) {
    rotationMotor.setControl(positionRequest.withPosition(Constants.getSideIntakeRotationsFromDegrees(position)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
