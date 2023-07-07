package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;


public class AutoBalance extends CommandBase {

  private Peripherals peripherals;
  private Drive drive;
  private PID pid;
  private double kP = 0.2;
  private double kI = 0;
  private double kD = 0.01;
  private double set;
  private double setPoint;
  private double turn;
  private double direction = 0;
  private boolean balanced = true;
  private double timer;
  private boolean timerStarted = false;
  Vector closerBalanceVector = new Vector(-0.2, 0.0);
  Vector fartherBalanceVector = new Vector(0.2, 0.0);
  Vector stopVector = new Vector(0.0, 0.0);
  Vector balanceVector;
  public AutoBalance(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    this.pid = pid;
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {
    drive.autoRobotCentricDrive(stopVector, 0);
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

    if(peripherals.getNavxRollOffset() > 5) {
      balanceVector = closerBalanceVector;
      balanced = false;
    } else if(peripherals.getNavxRollOffset() < -5) {
      balanceVector = fartherBalanceVector;
      balanced = false;
    } else {
      balanceVector = stopVector;
      balanced = true;
      timerStarted = false;
    }

    if(peripherals.getNavxRollOffset() > -7 && peripherals.getNavxRollOffset() < 7) {
      if(!timerStarted) {
        timer = Timer.getFPGATimestamp();
        timerStarted = true;
      }
    }

    if(timerStarted && Timer.getFPGATimestamp() - timer < 1.3) {
      balanceVector = stopVector;
    }


    drive.autoRobotCentricDrive(balanceVector, -result);
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoRobotCentricDrive(stopVector, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}