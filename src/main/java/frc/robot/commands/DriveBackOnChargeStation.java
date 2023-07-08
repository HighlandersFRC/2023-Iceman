package frc.robot.commands;

import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class DriveBackOnChargeStation extends CommandBase {
  private Peripherals peripherals;
  private Drive drive;
  private PID pid;
  private boolean checkpoint = false;
  private double startTimeOnStation = 0;
  private boolean balanced = false;
  private double kP = 0.2;
  private double kI = 0;
  private double kD = 0.01;
  private double set;
  private double setPoint;
  private double turn;

  Vector driveVector = new Vector(-1, 0);
  Vector balanceVector = new Vector(-0.45, 0.0);
  Vector stopVector = new Vector(0.0, 0.0);

  public DriveBackOnChargeStation(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    this.pid = pid;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    drive.autoRobotCentricDrive(driveVector, 0);
    pid = new PID(kP, kI, kD);
    if (drive.getFieldSide() == "red"){
      setPoint = 180;
    } else {
      setPoint = 0;
    }
    set = setPoint;
    pid.setSetPoint(setPoint);
    pid.setMinOutput(-1);
    pid.setMaxOutput(1);
  }

  @Override
  public void execute() {
    turn = peripherals.getNavxAngle();
    pid.updatePID(turn);
    System.out.println("offset " + peripherals.getNavxRollOffset());
    double result = pid.getResult();
    if(Math.abs(turn - set) < 2) { 
      result = 0;
    }
    drive.autoRobotCentricDrive(driveVector, -result);
    SmartDashboard.putBoolean("checkpoint 2", checkpoint);
    if(this.peripherals != null) {
      if(peripherals.getNavxRollOffset() > 12 && !checkpoint) {
        checkpoint = true;
        startTimeOnStation = Timer.getFPGATimestamp();
      }

      if(checkpoint && Timer.getFPGATimestamp() - startTimeOnStation > 1.3) {
        drive.autoRobotCentricDrive(balanceVector, 0);
      }

      if(checkpoint && peripherals.getNavxRollOffset() < 10 && !balanced) {
        balanced = true;
      }

    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    return balanced;
  }
}