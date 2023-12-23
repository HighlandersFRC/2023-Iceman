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

  private final WPI_TalonFX rotationMotorMaster = new WPI_TalonFX(11, "Canivore");
  private final WPI_TalonFX rotationMotorFollower = new WPI_TalonFX(12, "Canivore");
  
  private final CANCoder armRotationCancoder = new CANCoder(5, "Canivore");

  private ArmExtension armExtension;
  private FlipChecker flipChecker;

  public ArmRotation(ArmExtension armExtension, FlipChecker flipChecker) {
    this.armExtension = armExtension;
    this.flipChecker = flipChecker;
  }

  public void init() {
    rotationMotorMaster.configFactoryDefault();
    rotationMotorFollower.configFactoryDefault();

    rotationMotorMaster.configMotionCruiseVelocity(Constants.MAX_FALCON_TICS_PER_SECOND);
    rotationMotorMaster.configMotionAcceleration(Constants.MAX_FALCON_TICS_PER_SECOND_PER_SECOND);
    rotationMotorMaster.configMotionSCurveStrength(6);

    rotationMotorMaster.config_kP(0, 2.6);
    rotationMotorMaster.config_kI(0, 0.0);
    rotationMotorMaster.config_kD(0, 17);

    rotationMotorFollower.set(ControlMode.Follower, 11);

    rotationMotorMaster.setNeutralMode(NeutralMode.Coast);
    rotationMotorFollower.setNeutralMode(NeutralMode.Coast);

    rotationMotorMaster.configRemoteFeedbackFilter(armRotationCancoder, 0);
    rotationMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

    setDefaultCommand(new ArmRotationDefaultCommand(this, armExtension, flipChecker));
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
    SmartDashboard.putNumber("Setpoint bubblegum", degrees);
    rotationMotorMaster.set(ControlMode.MotionMagic, Constants.convertArmRotationDegreesToTics(degrees));
    // rotationMotorMaster.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}