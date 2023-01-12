// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class AlignToConeVertical extends CommandBase {
  /** Creates a new AlignToConeVertical. */
  private Drive drive;
  private Peripherals peripherals;

  private PID pid;
  private PID turnPID;

  // private double kP = 6;
  // private double kI = 0.15;
  // private double kD = 0.6;

  private double kP = 8;
  private double kI = 0;
  private double kD = 0.8;

  private double turnP = 10;
  private double turnI = 0;
  private double turnD = 0;

  public AlignToConeVertical(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-1);
    pid.setMaxOutput(1);

    turnPID = new PID(turnP, turnI, turnD);
    turnPID.setSetPoint(0);
    turnPID.setMinOutput(-0.5);
    turnPID.setMaxOutput(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double movement = peripherals.getLimeLightY();
    double turn = peripherals.getLimeLightX();
    turnPID.updatePID(turn);
    pid.updatePID(movement);
    double result = -pid.getResult();
    double turnResult = -turnPID.getResult();
    System.out.println("RESULT:   " + result);
    System.out.println("movement: " + movement);
    drive.autoDrive(new Vector(result, turnResult), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.autoDrive(new Vector(0, 0), 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(peripherals.getLimeLightY()) < Math.toRadians(0.5)) {
      return true;
    }
    return false;
  }
}
