// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Lights.LEDMode;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class AprilTagAlignment extends CommandBase {
  /** Creates a new VisionAlignment. */
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;

  private PID pid;
  private PID rotationPID;

  // private double kP = 1.25;
  // private double kI = 0.05;
  // private double kD = 1;

  private double kP = 3;
  private double kI = 0;
  private double kD = 0;

  private double rotationP = 0.27;
  private double rotationI = 0;
  private double rotationD = 0;
  
  // private double kP = 1.0;
  // private double kI = 0.0;
  // private double kD = 0.0;

  private int angleSettled = 0;

  private double startTime;

  public AprilTagAlignment(Drive drive, Peripherals peripherals, Lights lights) {
    this.peripherals = peripherals;
    this.drive = drive;
    this.lights = lights;
    addRequirements(this.drive, this.peripherals, this.lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-2);
    pid.setMaxOutput(2);

    rotationPID = new PID(rotationP, rotationI, rotationD);
    if (drive.getFieldSide() == "red"){
      rotationPID.setSetPoint(180);
      // rotationPID.setSetPoint(0);
    } else {
      rotationPID.setSetPoint(0);
    }
    rotationPID.setMinOutput(-3);
    rotationPID.setMaxOutput(3);

    angleSettled = 0;
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = peripherals.getBackLimelightAngleToTarget();

    if (currentAngle == 0){
      lights.setAMode(LEDMode.GOLDSTROBE);
      lights.setSMode(LEDMode.GOLDSTROBE);
    } else {
      lights.setAMode(LEDMode.GREEN);
      lights.setSMode(LEDMode.GREEN);
    }

    pid.updatePID(currentAngle);
    double result = -pid.getResult();

    double turn = peripherals.getNavxAngle();
    rotationPID.updatePID(turn);
    double rotationResult = -rotationPID.getResult();

    // if(peripherals.getFrontTargetArea() < 2.5) {
    drive.autoRobotCentricDrive(new Vector(-2.5, result * 1.25), rotationResult);
    System.out.println("Result: " + result);
    System.out.println("Angle: " + currentAngle);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ENDED");
    drive.autoRobotCentricDrive(new Vector(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - startTime > 0.5) {
      return true;
    }
    if(peripherals.getBackLimelightAngleToTarget() == 0) {
      return true;
    }
    return false;
  }
}
