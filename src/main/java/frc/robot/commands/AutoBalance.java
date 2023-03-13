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
  private double movingRollMargin = 1;
  private double speed = 0.8;
  private double timeSinceStoppedMoving = 0.0;
  private double pauseTime = 0.2;
  private double balancedRoll = 3.93;

  public AutoBalance(Drive drive, double speed) {
    this.drive = drive;
    this.speed = Math.abs(speed);
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
    System.out.println("Navx: " + roll);
    System.out.println("Roll Dif" + this.rollDif);
    if (Math.abs(this.rollDif) > this.movingRollMargin){
      drive.autoRobotCentricDrive(new Vector(0, 0), 0);
      this.timeSinceStoppedMoving = Timer.getFPGATimestamp();
    } else {
      if (this.roll >= this.balancedRoll + this.rollMargin && Timer.getFPGATimestamp() - this.timeSinceStoppedMoving > this.pauseTime){
        drive.autoRobotCentricDrive(new Vector(-speed, 0), 0);
      } else if (this.roll <= this.balancedRoll - this.rollMargin && Timer.getFPGATimestamp() - this.timeSinceStoppedMoving > this.pauseTime){
        drive.autoRobotCentricDrive(new Vector(speed, 0), 0);
      } else {
        drive.autoRobotCentricDrive(new Vector(0, 0), 0);
        this.timeSinceStoppedMoving = Timer.getFPGATimestamp();
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
    if (Math.abs(this.rollDif) < this.movingRollMargin && Math.abs(this.roll - this.balancedRoll) < this.rollMargin){
      return true;
    // } else if (drive.getCurrentTime() > 14.9 && drive.getCurrentTime() < 25) {
    //   return true;
    } else {
      return false;
    }
  }
}
