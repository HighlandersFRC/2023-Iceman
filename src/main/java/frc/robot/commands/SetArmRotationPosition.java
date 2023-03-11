// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.PRESET;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Wrist;

public class SetArmRotationPosition extends CommandBase {
  private ArmRotation arm;
  private double position = 180;

  private FlipChecker flipChecker;
  private boolean useNavx = false;
  private PRESET preset;
  private Peripherals peripherals;

  public SetArmRotationPosition(ArmRotation arm, FlipChecker flipChecker, double position) {
    this.arm = arm;
    this.position = position;
    this.flipChecker = flipChecker;
    this.useNavx = false;
    addRequirements(this.arm);
  }

  public SetArmRotationPosition(ArmRotation arm, Peripherals peripherals, FlipChecker flipChecker, PRESET preset){
    this.arm = arm;
    this.flipChecker = flipChecker;
    this.useNavx = true;
    this.preset = preset;
    this.peripherals = peripherals;
    addRequirements(this.arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double setPosition = position;
    
    if (useNavx){
      double angle = peripherals.getNavxAngle() % 360;
      while (angle < 0) {
        angle += 360;
      }
      if (angle >= 90 && angle <= 270){
        // Case where robot is facing towards driver
        switch(preset) {
          case HIGH_PLACEMENT:
            this.position = Constants.HIGH_PLACEMENT_ARM_ROTATION;
            break;
          case MID_PLACEMENT:
            this.position = Constants.MID_PLACEMENT_ARM_ROTATION;
            break;
          case UPRIGHT_CONE:
            this.position = Constants.UPRIGHT_CONE_ARM_ROTATION;
            break;
          case TIPPED_CONE:
            this.position = Constants.TIPPED_CONE_ARM_ROTATION;
            break;
          case CUBE:
            this.position = Constants.CUBE_ARM_ROTATION;
            break;
          case SHELF:
            this.position = Constants.SHELF_ARM_ROTATION;
            break;
          default:
            this.position = 180;
        }
      } else {
        // Case where robot is facing away from driver
        switch(preset) {
          case HIGH_PLACEMENT:
            this.position = Constants.HIGH_PLACEMENT_FLIPPED_ARM_ROTATION;
            break;
          case MID_PLACEMENT:
            this.position = Constants.MID_PLACEMENT_FLIPPED_ARM_ROTATION;
            break;
          case UPRIGHT_CONE:
            this.position = Constants.UPRIGHT_CONE_FLIPPED_ARM_ROTATION;
            break;
          case TIPPED_CONE:
            this.position = Constants.TIPPED_CONE_FLIPPED_ARM_ROTATION;
            break;
          case CUBE:
            this.position = Constants.CUBE_FLIPPED_ARM_ROTATION;
            break;
          case SHELF:
            this.position = Constants.SHELF_FLIPPED_ARM_ROTATION;
            break;
          default:
            this.position = 180;
        }
      }
    } else {
      if(position == Constants.UPRIGHT_CONE_ARM_ROTATION) {
        if(flipChecker.getFlip()) {
          setPosition = Constants.UPRIGHT_CONE_FLIPPED_ARM_ROTATION;
        }
      }
      else if(position == Constants.TIPPED_CONE_ARM_ROTATION) {
        if(flipChecker.getFlip()) {
          setPosition = Constants.TIPPED_CONE_FLIPPED_ARM_ROTATION;
        }
      }
      else if(position == Constants.MID_PLACEMENT_ARM_ROTATION) {
        if(flipChecker.getFlip()) {
          setPosition = Constants.MID_PLACEMENT_FLIPPED_ARM_ROTATION;
        }
      }
      else if(position == Constants.HIGH_PLACEMENT_ARM_ROTATION) {
        if(flipChecker.getFlip()) {
          setPosition = Constants.HIGH_PLACEMENT_FLIPPED_ARM_ROTATION;
        }
      } else if (position == Constants.CUBE_ARM_ROTATION) {
        if (flipChecker.getFlip()) {
          setPosition = Constants.CUBE_FLIPPED_ARM_ROTATION;
        }
      } else if (position == Constants.SHELF_ARM_ROTATION) {
        if (flipChecker.getFlip()) {
          setPosition = Constants.SHELF_FLIPPED_ARM_ROTATION;
        }
      }
    }
    if (OI.operatorController.getRawAxis(2) > 0.1) {
      arm.setRotationMotorPercent((-OI.operatorController.getRawAxis(2)/4));
      this.position = arm.getRotationPosition();
      setPosition = arm.getRotationPosition();
    } else if(OI.operatorController.getRawAxis(3) > 0.1) {
      arm.setRotationMotorPercent((OI.operatorController.getRawAxis(3))/4);
      this.position = arm.getRotationPosition();
      setPosition = arm.getRotationPosition();
    } else {
      if (useNavx){
        arm.setRotationPosition(this.position);
      } else {
        arm.setRotationPosition(setPosition);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Math.abs(arm.getRotationPosition() - position) < 2.5) {
      return true;
    }
    return false;
  }
}
