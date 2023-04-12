// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Lights.LEDMode;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class DriveAutoAligned extends CommandBase {
  /** Creates a new DriveAutoAligned. */
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;
  private double turn = 0;

  private PID pid;

  private double set = 0;

  private double kP = 0.27;
  private double kI = 0;
  private double kD = 0;

  // private double kP = 0.5;
  // private double kI = 0;
  // private double kD = 2;

  private int angleSettled = 0;

  private double initTime = 0;
  public DriveAutoAligned(Drive drive, Peripherals peripherals, Lights lights) {
    this.peripherals = peripherals;
    this.drive = drive;
    this.lights = lights;
    addRequirements(this.drive, this.lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    int numTurns = ((int) peripherals.getNavxAngle())/180;
    double setPoint = 180 * Math.copySign(numTurns, peripherals.getNavxAngle());
    if((Math.abs(peripherals.getNavxAngle()) % 180) > 90) {
      if(peripherals.getNavxAngle() < 0) {
        setPoint = 180 * Math.copySign(numTurns - 1, peripherals.getNavxAngle());
      }
      else {
        setPoint = 180 * Math.copySign(numTurns + 1, peripherals.getNavxAngle());
      }
    }
      set = setPoint;
    pid.setSetPoint(setPoint);
    pid.setMinOutput(-3);
    pid.setMaxOutput(3);
    angleSettled = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turn = peripherals.getNavxAngle();
    pid.updatePID(turn);
    double result = -pid.getResult();
    System.out.println(result);
    if(Math.abs(turn - set) < 2) { 
      result = 0;
    }
    drive.driveAutoAligned(result);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drive.autoDrive(new Vector(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(!OI.driverController.getXButton()) {
    //   return true;
    // }
    return false;
  }
}