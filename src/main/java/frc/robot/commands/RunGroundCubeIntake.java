// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.GroundCubeIntake;

public class RunGroundCubeIntake extends CommandBase {
  /** Creates a new RunGroundCubeIntake. */
  private GroundCubeIntake intake;
  private boolean hasSpunUp = false;

  private double position;
  private double speed;
  public RunGroundCubeIntake(GroundCubeIntake intake, double position, double speed) {
    this.intake = intake;
    this.position = position;
    this.speed = speed;
    addRequirements(this.intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speed > 0) {
      if(intake.getPosition() > 80) {
        intake.setIntakeRotationPercent(15, 0.2);
      }
      else {
        intake.setIntakeRotation(position);
      }
    }

    SmartDashboard.putNumber("Ground intake position", intake.getPosition());

    double intakeVelocity = Math.abs(intake.getVelocity());

      if(intakeVelocity > 20) {
        hasSpunUp = true;
        intake.setIntakeTorqueOutput((55) * Math.signum(speed), Math.abs(speed));
      }
      if(intakeVelocity < 20 && hasSpunUp) {
        intake.setIntakeTorqueOutput((55) * Math.signum(speed), Math.abs(speed));
        // set lights and rumble
        OI.driverController.setRumble(RumbleType.kBothRumble, 1);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 1);
      }
      else {
        intake.setIntakeTorqueOutput((55) * Math.signum(speed), Math.abs(speed));
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
