// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;

public class ArmRotationDefaultCommand extends CommandBase {
  /** Creates a new ArmDefaultCommand. */
  private ArmRotation arm;
  private ArmExtension armExtension;
  double position = 0;
  public ArmRotationDefaultCommand(ArmRotation arm, ArmExtension armExtension) {
    this.arm = arm;
    this.armExtension = armExtension;
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SD Rotation", 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if((OI.operatorController.getRawAxis(4))/4 < 0.0125) {
      if(armExtension.getExtensionPosition() < 5) {
        position = SmartDashboard.getNumber("SD Rotation", 180);
        // if(position > 90 && position < 250) {
        //   arm.setRotationPosition(position);
        // }
        // else {
        arm.setRotationPosition(180);
        // if(OI.operatorController.getLeftTriggerAxis() > 0.25) {
        //   arm.setRotationMotorPercent(OI.operatorController.getLeftTriggerAxis()/4);
        // }
        // if(OI.operatorController.getRightTriggerAxis() > 0.25) {
        //   arm.setRotationMotorPercent(-OI.operatorController.getRightTriggerAxis()/4);
        // }
        // else {
        //   arm.setRotationMotorPercent(0);
        // }
        // }
      }
    // }
    // else {
    //   arm.setRotationMotorPercent((OI.operatorController.getRawAxis(4))/4);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
