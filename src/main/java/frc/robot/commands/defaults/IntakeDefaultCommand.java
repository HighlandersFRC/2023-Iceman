// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Intake;

public class IntakeDefaultCommand extends CommandBase {
  /** Creates a new IntakeDefaultCommand. */
  private Intake Intake;
  public IntakeDefaultCommand(Intake Intake) {
    this.Intake = Intake;
    addRequirements(this.Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(Intake.getGrabberMotorCurrent()) < 25) {
    //   this.Intake.setGrabberMotorPercent(0.1);
    // }
    // else {
    //   Intake.setGrabberMotorPercent(0);
    // }
    if(OI.driverController.getLeftTriggerAxis() > 0.5) {
      Intake.setIntakeTorqueOutput(55, 1);
      // System.out.println("OUTTAKING");
    }
    else if(OI.driverController.getRightTriggerAxis() > 0.5) {
      Intake.setIntakeTorqueOutput(-55, 1);
      // System.out.println("INTAKING");
    }
    else {
      Intake.setIntakeTorqueOutput(-35, 0.10);
    }
    // Intake.setIntakeRotationPosition(150);
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
