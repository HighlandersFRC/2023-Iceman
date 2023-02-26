// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.FlipChecker;
import frc.robot.subsystems.Wrist;

public class SetArmRotationPosition extends CommandBase {
  /** Creates a new SetArmRotationPosition. */
  private ArmRotation arm;
  private double position;

  private FlipChecker flipChecker;

  public SetArmRotationPosition(ArmRotation arm, FlipChecker flipChecker, double Position) {
    this.arm = arm;
    this.position = Position;
    this.flipChecker = flipChecker;
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // upright cone and check to see which side to intake on
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Position: " + position);
    double setPosition = position;
    if(position == Constants.UPRIGHT_CONE_ARM_ROTATION) {
      if(flipChecker.getFlip()) {
        setPosition = Constants.UPRIGHT_CONE_FLIPPED_ARM_ROTATION;
      }
    }
    // tipped over cone and check to see which side to intake on
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
      System.out.println("ajsdfklajsdklfja;sdklfja;klsdjflk");
      if(flipChecker.getFlip()) {
        setPosition = Constants.HIGH_PLACEMENT_FLIPPED_ARM_ROTATION;
      }
    }
    if(OI.operatorController.getLeftTriggerAxis() > 0.1) {
      arm.setRotationMotorPercent((-OI.operatorController.getLeftTriggerAxis()/4));
    }
    else if(OI.operatorController.getRightTriggerAxis() > 0.1) {
      arm.setRotationMotorPercent((OI.operatorController.getLeftTriggerAxis())/4);
    }
    else {
      arm.setRotationPosition(setPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(arm.getRotationPosition() - position) < 2.5) {
      return true;
    }
    return false;
  }
}
