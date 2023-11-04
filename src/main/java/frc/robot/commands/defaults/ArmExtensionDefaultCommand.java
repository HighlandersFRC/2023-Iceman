// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;

public class ArmExtensionDefaultCommand extends CommandBase {
  /** Creates a new ArmExtensionDefaultCommand. */
  private ArmExtension arm;
  double position;
  public ArmExtensionDefaultCommand(ArmExtension arm) {
    this.arm = arm;
    addRequirements(this.arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SD Extension", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.getExtensionLimitSwitch()) {
      arm.setExtensionEncoderPosition(0);
      // System.out.println("TRUE");
    } else {
      // System.out.println("FALSE");
    }
    // position = SmartDashboard.getNumber("SD Extension", 0);
    // if(position > 0 && position < 30) {
    //   arm.setExtensionPosition(position);
    // }
    // else {
    // // arm.setExtensionPosition(0);
    // arm.setExtensionMotorPercent(position);
    // }

    // if(arm.getExtensionPosition() > 10) {
    //   arm.setExtensionPosition(5);
    // }
    // else {
    //   arm.setExtensionPosition(0);
    // }
    arm.setExtensionPosition(2);
    // arm.setExtensionMotorPercent(0.0);
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
