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

public class MoveToPieceForwards extends CommandBase {
  /** Creates a new VisionAlignment. */
  private Drive drive;
  private Peripherals peripherals;
  private Lights lights;
  private double turn = 0;

  private PID pid;

  // private double kP = 1.25;
  // private double kI = 0.05;
  // private double kD = 1;

  private double kP = 6;
  private double kI = 0;
  private double kD = 0;
  
  // private double kP = 1.0;
  // private double kI = 0.0;
  // private double kD = 0.0;

  private int angleSettled = 0;

  private double startTime;

  private double initialAngle = 0;
  private double target = 0;

  private double initTime = 0;
  public MoveToPieceForwards(Drive drive, Peripherals peripherals, Lights lights) {
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
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
    angleSettled = 0;
    startTime = Timer.getFPGATimestamp();
    // peripherals.setFrontPipeline(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Pipeline: " + peripherals.getFrontLimelightPipeline());
    double currentAngle = peripherals.getFrontLimelightAngleToTarget();
    // System.out.println("Angle: " + currentAngle);
    // System.out.println("JSON: " + peripherals.getFrontJSONString());
    if (currentAngle == 0){
      lights.setAMode(LEDMode.GOLDSTROBE);
      lights.setSMode(LEDMode.GOLDSTROBE);
    } else {
      lights.setAMode(LEDMode.GREEN);
      lights.setSMode(LEDMode.GREEN);
    }
    pid.updatePID(currentAngle);
    double result = -pid.getResult();
    SmartDashboard.putNumber("Angle Settled", angleSettled);

    // if(peripherals.getFrontTargetArea() < 2.5) {
    drive.autoRobotCentricDrive(new Vector(1.75 * 1.5, 0), result * 1.5);
    System.out.println("Result " + result);
    System.out.println("Angle: " + currentAngle);
    double[] vels = drive.getWheelVelocities();
    double[] outs = drive.getWheelOutputs();
    System.out.println("------------------------");
    System.out.println("M1 Set: " + Math.round(outs[0] * 100.0) / 100.0 + " Vel: " + Math.round(vels[0] * 100) / 100.0);
    System.out.println("------------------------");
    System.out.println("M2 Set: " + Math.round(outs[1] * 100.0) / 100.0 + " Vel: " + Math.round(vels[1] * 100) / 100.0);
    System.out.println("------------------------");
    System.out.println("M3 Set: " + Math.round(outs[2] * 100.0) / 100.0 + " Vel: " + Math.round(vels[2] * 100) / 100.0);
    System.out.println("------------------------");
    System.out.println("M4 Set: " + Math.round(outs[3] * 100.0) / 100.0 + " Vel: " + Math.round(vels[3] * 100) / 100.0);
    System.out.println("------------------------");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(new Vector(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - startTime > 0.7) {
      return true;
    }
    return false;
  }
}
