// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class IntakeDefaultCommand extends CommandBase {
  /** Creates a new IntakeDefaultCommand. */
  private Intake Intake;
  private boolean hasSpunUp = false;
  private Lights lights;
  public IntakeDefaultCommand(Intake Intake, Lights lights) {
    this.Intake = Intake;
    this.lights = lights;
    addRequirements(this.Intake, this.lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 1);
    double intakeVelocity = Math.abs(Intake.getVelocity());
    if(OI.driverController.getLeftTriggerAxis() > 0.5) {
      Intake.setIntakeTorqueOutput(55, 1);
      // System.out.println("OUTTAKING");
    }
    else if(OI.driverController.getRightTriggerAxis() > 0.5) {
      Intake.setIntakeTorqueOutput(-55, 1);
      if(intakeVelocity > 20) {
        hasSpunUp = true;
      }
      if(intakeVelocity < 15 && hasSpunUp) {
        hasSpunUp = false;
        // set lights and rumble
        lights.setLightsToIntake();
        OI.driverController.setRumble(RumbleType.kBothRumble, 1);
        OI.operatorController.setRumble(RumbleType.kBothRumble, 1);
      }
    }
    else {
      hasSpunUp = false;
      Intake.setIntakeTorqueOutput(-45, 0.10);
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
      OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
      lights.setCorrectMode();
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
