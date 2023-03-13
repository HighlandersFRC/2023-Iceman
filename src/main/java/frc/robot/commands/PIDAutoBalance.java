// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class PIDAutoBalance extends CommandBase {
  private Drive drive;
  private Peripherals peripherals;
  private PID pid;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private double balancedRoll = 3.93;
  private double rollMargin = 1;

  /** Creates a new PIDAutoBalance. */
  public PIDAutoBalance(Drive drive, Peripherals peripherals) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.peripherals = peripherals;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pid = new PID(this.kP, this.kI, this.kD);
    this.pid.setSetPoint(this.balancedRoll);
    this.pid.setMaxInput(0.5);
    this.pid.setMinInput(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roll = this.peripherals.getNavxRoll();
    this.pid.updatePID(roll);
    if (Math.abs(this.balancedRoll - roll) < this.rollMargin){
      drive.autoRobotCentricDrive(new Vector(0, 0), 0);
    } else {
      drive.autoRobotCentricDrive(new Vector(this.pid.getResult(), 0), 0);
    }
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
