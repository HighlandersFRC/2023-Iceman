// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Lights;

public class SetArmExtensionPosition extends CommandBase {
  /** Creates a new SetArmExtensionPosition. */
  private Lights lights;
  private ArmExtension armExtension;
  private ArmRotation armRotation;
  private double position;
  private boolean clearLights = false;

  public SetArmExtensionPosition(Lights lights, ArmExtension armExtension, ArmRotation armRotation, double position) {
    this.lights = lights;
    this.armExtension = armExtension;
    this.armRotation = armRotation;
    this.position = position;
    addRequirements(this.armExtension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public SetArmExtensionPosition(Lights lights, ArmExtension armExtension, ArmRotation armRotation, double position, boolean clearLights){
    this.lights = lights;
    this.armExtension = armExtension;
    this.armRotation = armRotation;
    this.position = position;
    this.clearLights = clearLights;
    addRequirements(this.armExtension);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this.clearLights){
      this.lights.setLightMode("side");
    }
    if(armRotation.getRotationPosition() < 100 || armRotation.getRotationPosition() > 260) {
      armExtension.setExtensionPosition(0);
    }
    else {
      armExtension.setExtensionPosition(position);
    }
    // System.out.println("Extension Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Extension Execute");
    if(armRotation.getRotationPosition() < 100 || armRotation.getRotationPosition() > 260) {
      armExtension.setExtensionPosition(0);
    }
    else {
      armExtension.setExtensionPosition(position);
    }
    SmartDashboard.putNumber("tics", armExtension.getExtensionRawPosition());
    SmartDashboard.putNumber("Extension Current", armExtension.getExtensionCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Extension End, Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("Arm Extension Error: " + Math.abs(armExtension.getExtensionPosition() - position));
    if(Math.abs(armExtension.getExtensionPosition() - position) < 2) {
      return true;
    }
    return false;
  }
}
