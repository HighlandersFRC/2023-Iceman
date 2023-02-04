// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristDefaultCommand extends CommandBase {
  /** Creates a new WristDefaultCommand. */
  private Wrist Wrist;
  public WristDefaultCommand(Wrist Wrist) {
    this.Wrist = Wrist;
    addRequirements(this.Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(Wrist.getGrabberMotorCurrent()) < 25) {
    //   this.Wrist.setGrabberMotorPercent(0.1);
    // }
    // else {
    //   Wrist.setGrabberMotorPercent(0);
    // }
    Wrist.setGrabberMotorPercent(0);
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
