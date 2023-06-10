// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotionMagicConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.Follower;
import com.ctre.phoenixpro.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ArmExtensionDefaultCommand;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;

public class ArmExtension extends SubsystemBase {

  private final TalonFX extensionMotor = new TalonFX(10, "Canivore");
  private final TalonFX followerExtensionMotor = new TalonFX(18, "Canivore");

  private TalonFXConfigurator configurator = extensionMotor.getConfigurator();

  private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  private CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs();

  private final VoltageOut percentRequest = new VoltageOut(0);
  private MotionMagicTorqueCurrentFOC motionMagicRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);

  
  /** Creates a new ArmExtension. */
  public ArmExtension() {}

  public void init() {
    configurator.apply(new TalonFXConfiguration());

    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    currentLimitConfigs.StatorCurrentLimitEnable = true;
    currentLimitConfigs.SupplyCurrentLimitEnable = true;

    currentLimitConfigs.StatorCurrentLimit = 80;
    currentLimitConfigs.SupplyCurrentLimit = 80;

    // motorOutputConfigs.DutyCycleNeutralDeadband = 0.125;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.5; // Add 0.05 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 17; // A position error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.MAX_FALCON_ROTATIONS_PER_SECOND;
    motionMagicConfigs.MotionMagicAcceleration = Constants.MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND;
    motionMagicConfigs.MotionMagicJerk = Constants.MAX_FALCON_ROTATIONS_PER_SECOND_PER_SECOND_PER_SECOND;

    configurator.apply(motorOutputConfigs);
    configurator.apply(motionMagicConfigs);
    configurator.apply(slot0Configs);
    configurator.apply(currentLimitConfigs);

    followerExtensionMotor.setControl(new Follower(10, false));

    setDefaultCommand(new ArmExtensionDefaultCommand(this));
  }

  public void teleopInit() {
    // extensionMotor.configPeakOutputReverse(-0.25);
  }

  public double getExtensionCurrent() {
    return extensionMotor.getStatorCurrent().getValue();
  }

  public double getExtensionPosition() {
    return Constants.getArmExtensionInchesFromRotations(extensionMotor.getRotorPosition().getValue());
  }

  public double getExtensionRawPosition() {
    return extensionMotor.getRotorPosition().getValue();
  }

  public boolean getExtensionLimitSwitch() {
    if(extensionMotor.getReverseLimit().getValue().value == 1) {
      return false;
    }
    return true;
  }

  public void setExtensionEncoderPosition(double inches) {
    extensionMotor.setRotorPosition(Constants.getArmExtensionRotations(inches));
  }

  public void setExtensionMotorPercent(double percent) {
    extensionMotor.setControl(this.percentRequest.withOutput(percent));
  }

  public void setExtensionPosition(double inches) {
    SmartDashboard.putNumber("Desired", Constants.getArmExtensionTics(inches));
    // extensionMotor.setControl(tpercentRequest);
    extensionMotor.setControl(motionMagicRequest.withPosition(Constants.getArmExtensionRotations(inches)));
    
    // extensionMotor.set(ControlMode.Position, Constants.getArmExtensionTics(inches));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


