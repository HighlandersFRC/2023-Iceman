// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;
import frc.robot.tools.controlloops.PID;

public class WristDefaultCommand extends CommandBase {
  /** Creates a new WristDefaultCommand. */
  private Wrist wrist;
  // private PID pid;

  // private double kP = 0.02;
  // private double kI = 0.0015;
  // private double kD = 0.0;

  public WristDefaultCommand(Wrist wrist) {
    this.wrist = wrist;
    addRequirements(this.wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pid = new PID(kP, kI, kD);
    // pid.setSetPoint(180);
    // pid.setMaxOutput(0.5);
    // pid.setMinOutput(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double angle = wrist.getWristRotationPosition();
    // pid.updatePID(angle);
    // double result = pid.getResult();
    // if (Math.abs(angle - 180) < 2){
    //   wrist.setRotationMotorPercent(0);
    // } else {
    //   wrist.setRotationMotorPercent(-result);
    // }
    wrist.setRotationPosition(180);
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
