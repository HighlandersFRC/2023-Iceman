// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.tools.math.Vector;

import java.lang.Math;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private Drive drive;
  private double roll;
  private double rollDif;
  // acceptable margin of roll error from level
  private double rollMargin = 5;
  private double speed = 0.8;

  public AutoBalance(Drive drive, double speed) {
    this.drive = drive;
    this.speed = speed;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.rollDif = drive.getNavxRoll() - this.roll;
    this.roll = drive.getNavxRoll();
    System.out.println("Navx Roll: " + this.roll);
    if (Math.abs(this.rollDif) > 3){
      System.out.println("Moving");
      drive.autoRobotCentricDrive(new Vector(0, 0), 0);
    } else {
      if (this.roll >= -2.74 + this.rollMargin){
        System.out.println("Backward");
        drive.autoRobotCentricDrive(new Vector(-speed, 0), 0);
      } else if (this.roll <= -2.74 - this.rollMargin){
        System.out.println("Forward");
        drive.autoRobotCentricDrive(new Vector(speed, 0), 0);
      } else {
        drive.autoRobotCentricDrive(new Vector(0, 0), 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(new Vector(0, 0), 0);
    drive.setWheelsHorizontal();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(this.rollDif) < 0.25 && Math.abs(this.roll + 2.74) < this.rollMargin){
      return true;
    } else if (drive.getCurrentTime() > 14.9 && drive.getCurrentTime() < 17) {
      return true;
    } else {
      return false;
    }
  }
}
