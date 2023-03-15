// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class AprilTagBalance extends CommandBase {
  /** Creates a new AprilTagBalance. */
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;
  
  private PID pid;
  private double kP = 3.5;
  private double kI = 0.0;
  private double kD = 0.0;

  // horizontal (x) distance from apriltag to center of charging station
  private double desiredDistance;

  // side specific distances
  private double desiredBlueSideDistance = 2.86;
  private double desiredRedSideDistance = 2.86;

  // horizontal (x) acceptable margin of distance error
  private double distanceMargin = 0.03;

  // level roll (deg.)
  private double balancedRoll = 3.93;

  // acceptable margin of roll error
  private double rollMargin = 3;

  // maximum speed (m/s)
  private double speed;

  // number of code cycles the distance has been within margin
  private int numCyclesWithinMargin = 0;

  // number of code cycles the distance has to be within margin in order for command to end
  private int distCycleMargin = 2;

  public AprilTagBalance(Drive drive, Peripherals peripherals, Lights Lights, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.peripherals = peripherals;
    this.lights = lights;
    this.speed = Math.abs(speed);

    if (this.drive.getFieldSide() == "blue"){
      this.desiredDistance = this.desiredBlueSideDistance;
    } else {
      this.desiredDistance = this.desiredRedSideDistance;
    }

    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pid = new PID(kP, kI, kD);
    this.pid.setSetPoint(this.desiredDistance);
    this.pid.setMaxOutput(this.speed);
    this.pid.setMinOutput(-this.speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double dist = peripherals.getBackHorizontalDistToTag();
    if (dist == 0){
      double roll = peripherals.getNavxRoll();
      if (roll >= this.balancedRoll + this.rollMargin){
        drive.autoRobotCentricDrive(new Vector(-this.speed / 2, 0), 0);
      } else if (roll <= this.balancedRoll - this.rollMargin){
        drive.autoRobotCentricDrive(new Vector(this.speed / 2, 0), 0);
      } else {
        drive.autoRobotCentricDrive(new Vector(0, 0), 0);
      }
    } else {
      this.pid.updatePID(dist);
      double result = this.pid.getResult();
      drive.autoRobotCentricDrive(new Vector(result, 0), 0);
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
    System.out.println("April Tag Dist: " + peripherals.getBackHorizontalDistToTag());
    if (Math.abs(peripherals.getBackHorizontalDistToTag() - this.desiredDistance) <= this.distanceMargin){
      this.numCyclesWithinMargin ++;
    } else {
      this.numCyclesWithinMargin = 0;
    }
    if (this.numCyclesWithinMargin >= this.distCycleMargin){
      return true;
    } else {
      return false;
    }
  }
}
