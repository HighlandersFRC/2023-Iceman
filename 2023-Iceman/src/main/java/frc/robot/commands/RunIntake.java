// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  private Intake Intake;
  private double percent;
  private double time;
  private double initTime;
  public RunIntake(Intake Intake, double percent, double time) {
    this.Intake = Intake;
    this.percent = percent;
    this.time = time;
    addRequirements(this.Intake);
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
    // if(Math.abs(Intake.getGrabberMotorCurrent()) < 40) {
    //   this.Intake.setGrabberMotorPercent(percent);
    // }
    // else {
    //   Intake.setGrabberMotorPercent(0);
    // }
    Intake.setGrabberMotorPercent(percent);
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
