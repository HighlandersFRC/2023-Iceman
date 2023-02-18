// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.defaults.ArmRotationDefaultCommand;

public class ArmRotation extends SubsystemBase {
  /** Creates a new ArmRotation. */

  private final WPI_TalonFX rotationMotorMaster = new WPI_TalonFX(11);
  private final WPI_TalonFX rotationMotorFollower = new WPI_TalonFX(12);
  
  private final CANCoder armRotationCancoder = new CANCoder(5);

  private ArmExtension armExtension;

  public ArmRotation(ArmExtension armExtension) {
    this.armExtension = armExtension;
  }

  public void init() {
    rotationMotorMaster.configFactoryDefault();
    rotationMotorFollower.configFactoryDefault();
    rotationMotorMaster.configPeakOutputForward(0.6);
    rotationMotorMaster.configPeakOutputReverse(-0.6);

    rotationMotorMaster.config_kP(0, 4.5);
    rotationMotorMaster.config_kI(0, 0);
    rotationMotorMaster.config_kD(0, 6);

    rotationMotorFollower.set(ControlMode.Follower, 11);

    // rotationMotorFollower.follow(rotationMotorMaster);

    rotationMotorMaster.setNeutralMode(NeutralMode.Coast);
    rotationMotorFollower.setNeutralMode(NeutralMode.Coast);

    rotationMotorMaster.configRemoteFeedbackFilter(armRotationCancoder, 0);
    rotationMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    setDefaultCommand(new ArmRotationDefaultCommand(this, armExtension));
  }

  public double getRotationPosition() {
    return armRotationCancoder.getAbsolutePosition();
  }

  public void setRotationMotorPercent(double percent) {
    rotationMotorMaster.set(ControlMode.PercentOutput, percent);
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
