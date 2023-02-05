// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class RunWrist extends CommandBase {
  /** Creates a new RunIntake. */
  private Wrist Wrist;
  private double percent;
  private double time;
  private double initTime;
  public RunWrist(Wrist Wrist, double percent, double time) {
    this.Wrist = Wrist;
    this.percent = percent;
    this.time = time;
    addRequirements(this.Wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(Wrist.getGrabberMotorCurrent()) < 40) {
    //   this.Wrist.setGrabberMotorPercent(percent);
    // }
    // else {
    //   Wrist.setGrabberMotorPercent(0);
    // }
    Wrist.setGrabberMotorPercent(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((Timer.getFPGATimestamp() - initTime) > time && time != -1) {
      return true;
    }
    return false;
  }
}
