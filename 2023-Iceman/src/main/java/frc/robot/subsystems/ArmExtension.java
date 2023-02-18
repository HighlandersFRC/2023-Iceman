// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ArmExtensionDefaultCommand;

public class ArmExtension extends SubsystemBase {

  private final TalonFX extensionMotor = new TalonFX(10, "Canivore");
  
  /** Creates a new ArmExtension. */
  public ArmExtension() {}

  public void init() {
    extensionMotor.configFactoryDefault();
    
    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    extensionMotor.configForwardSoftLimitThreshold(Constants.getArmExtensionTics(Constants.MAX_EXTENSION));
    extensionMotor.configForwardSoftLimitEnable(true);

    extensionMotor.setSelectedSensorPosition(0);

    extensionMotor.configPeakOutputForward(0.5);
    extensionMotor.configPeakOutputReverse(-0.2);

    extensionMotor.configOpenloopRamp(0.5);

    extensionMotor.config_kP(0, 0.95);
    extensionMotor.config_kI(0, 0);
    extensionMotor.config_kD(0, 0);

    extensionMotor.config_IntegralZone(0, 1000);
    extensionMotor.configNeutralDeadband(0.15, 100);
    extensionMotor.configAllowableClosedloopError(0, 800, 100);

    extensionMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
    extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));
    
    setDefaultCommand(new ArmExtensionDefaultCommand(this));
  }

  public double getExtensionPosition() {
    return Constants.getArmExtensionInches(extensionMotor.getSelectedSensorPosition());
  }

  public double getExtensionRawPosition() {
    return extensionMotor.getSelectedSensorPosition();
  }

  public boolean getExtensionLimitSwitch() {
    if(extensionMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      return true;
    }
    return false;
  }

  public void setExtensionEncoderPosition(double inches) {
    extensionMotor.setSelectedSensorPosition(Constants.getArmExtensionTics(inches));
  }

  public void setExtensionMotorPercent(double percent) {
    extensionMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setExtensionPosition(double inches) {
    SmartDashboard.putNumber("Desired", Constants.getArmExtensionTics(inches));
    extensionMotor.set(ControlMode.Position, Constants.getArmExtensionTics(inches));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
