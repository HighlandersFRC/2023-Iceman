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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ArmDefaultCommand;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX extensionMotor = new WPI_TalonFX(10);
  private final WPI_TalonFX rotationMotorMaster = new WPI_TalonFX(11);
  private final WPI_TalonFX rotationMotorFollower = new WPI_TalonFX(12);
  
  private final CANCoder armRotationCancoder = new CANCoder(5);

  /** Creates a new Arm. */
  public Arm() {}

  public void init() {
    extensionMotor.configFactoryDefault();
    rotationMotorMaster.configFactoryDefault();
    rotationMotorFollower.configFactoryDefault();

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    extensionMotor.configClearPositionOnLimitR(false, 0);

    extensionMotor.configForwardSoftLimitThreshold(Constants.getArmExtensionTics(Constants.MAX_EXTENSION));
    extensionMotor.configForwardSoftLimitEnable(true);

    extensionMotor.setSelectedSensorPosition(0);

    rotationMotorMaster.config_kP(0, 10);
    rotationMotorMaster.config_kI(0, 0);
    rotationMotorMaster.config_kD(0, 4);

    extensionMotor.configPeakOutputForward(0.35);
    extensionMotor.configPeakOutputReverse(-0.35);

    extensionMotor.config_kP(0, 0.26);
    extensionMotor.config_kI(0, 0.0005);
    extensionMotor.config_kD(0, 0);

    extensionMotor.config_IntegralZone(0, 1000);
    extensionMotor.configNeutralDeadband(0.15, 100);
    extensionMotor.configAllowableClosedloopError(0, 800, 100);

    extensionMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 40, 0));
    extensionMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0));

    rotationMotorFollower.set(ControlMode.Follower, 11);

    // rotationMotorFollower.follow(rotationMotorMaster);

    rotationMotorMaster.setNeutralMode(NeutralMode.Coast);
    rotationMotorFollower.setNeutralMode(NeutralMode.Coast);

    rotationMotorMaster.configRemoteFeedbackFilter(armRotationCancoder, 0);
    rotationMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    
    setDefaultCommand(new ArmDefaultCommand(this));
  }

  public double getExtensionPosition() {
    return Constants.getArmExtensionInches(extensionMotor.getSelectedSensorPosition());
  }

  public double getExtensionRawPosition() {
    return extensionMotor.getSelectedSensorPosition();
  }

  public double getRotationPosition() {
    return armRotationCancoder.getAbsolutePosition();
  }

  public boolean getExtensionLimitSwitch() {
    if(extensionMotor.getSensorCollection().isRevLimitSwitchClosed() == 1) {
      return false;
    }
    return true;
  }

  public void setExtensionEncoderPosition(double inches) {
    extensionMotor.setSelectedSensorPosition(Constants.getArmExtensionTics(inches));
  }

  public void setRotationMotorPercent(double percent) {
    rotationMotorMaster.set(ControlMode.PercentOutput, percent);
  }

  public void setExtensionMotorPercent(double percent) {
    extensionMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setExtensionPosition(double inches) {
    SmartDashboard.putNumber("Desired", Constants.getArmExtensionTics(inches));
    extensionMotor.set(ControlMode.Position, Constants.getArmExtensionTics(inches));
  }

  public void postRotationValues() {
    SmartDashboard.putNumber("Setpoint", rotationMotorMaster.getClosedLoopTarget());
    SmartDashboard.putNumber("Closed loop error", rotationMotorMaster.getClosedLoopError());
    SmartDashboard.putNumber("Motor position", Constants.convertArmRotationTicsToDegrees(rotationMotorMaster.getSelectedSensorPosition()));
    SmartDashboard.putNumber("Arm Rotation Position", armRotationCancoder.getAbsolutePosition());
  }

  public void setRotationPosition(double degrees) {
    // SmartDashboard.putNumber("Setpoint", degrees);
    rotationMotorMaster.set(ControlMode.Position, Constants.convertArmRotationDegreesToTics(degrees));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
