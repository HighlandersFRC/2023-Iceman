package frc.robot.commands;

import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;

public class DriveOverChargeStation extends CommandBase {
  private Peripherals peripherals;
  private Drive drive;
  private PID pid;
  private boolean pastStation = false;
  private boolean checkpoint = false;
  private boolean exitingStation = false;
  private double timePastStation = 0;
  private double kP = 0.2;
  private double kI = 0;
  private double kD = 0.01;
  private double set;
  private double setPoint;
  private double turn;

  Vector driveVector = new Vector(2.1, 0);
  Vector stopVector = new Vector(0.0, 0.0);

  public DriveOverChargeStation(Drive drive, Peripherals peripherals) {
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
    double result = pid.getResult();
    if(Math.abs(turn - set) < 2) { 
      result = 0;
    }
    drive.autoRobotCentricDrive(driveVector, -result);
    if(peripherals.getNavxRollOffset() < -10) {
      checkpoint = true;
    }

    if(peripherals.getNavxRollOffset() > 5 && checkpoint && !exitingStation) {
      exitingStation = true;
    }

    if(peripherals.getNavxRollOffset() < 5 && exitingStation && !pastStation) {
      timePastStation = Timer.getFPGATimestamp();
      pastStation = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timePastStation > 0.3 && pastStation) {
      return true;
    } else {
      return false;
    }
  }
}